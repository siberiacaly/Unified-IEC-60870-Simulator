// =======================
// ZÁKLADNÍ INCLUDES
// =======================

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include "cs104_slave.h"
#include "hal_thread.h"
#include "hal_time.h"
#include "hal_serial.h"
#include "iec60870_common.h"
#include "lib_memory.h"
#include <setjmp.h>
#include "cs101_master.h"
#include "cs104_connection.h"
#include "cs101_slave.h"

// =======================
// KONSTANTY A GLOBÁLNÍ PROMĚNNÉ
// =======================

#define MAX_MESSAGES 100           // Max počet zpráv v konfiguraci
#define MAX_MESSAGE_TYPES 40       // Max počet typů zpráv (pro indexaci)
#define MAX_MESSAGE_CONFIGS 50
#define MAX_IO_CONTENTS 20

int numPermMessageConfigs = 0;
int numTempMessageConfigs = 0;
int dataConfig = 0;                // Přepínač datových logů (1=loguje)
int serviceConfig = 0;             // Přepínač servisních logů (1=loguje)

static char *dataPath = "DATALOG.txt";       // Výchozí cesta k datovým logům
static char *servicePath = "SERVICELOG.txt"; // Výchozí cesta k servisním logům

// =======================
// STRUKTURY PRO KONFIGURACI
// =======================

// IOContent - jeden datový bod (typicky jeden IOA a hodnota)
typedef struct {
    int ioa;
    float value;            // Pro 1-hodnotovou zprávu (TEMP, PERM-statická)
    float toggleValueA;     // Dual: první hodnota
    float toggleValueB;     // Dual: druhá hodnota
    bool toggleEnabled;     // True, pokud se má přepínat mezi dvěma hodnotami
    bool toggleState;       // Uchovává aktuální stav (true=A, false=B)
} IOContent;

// MessageConfig - jeden typ zprávy s polem IO bodů
typedef struct {
    int messageType;
    IOContent ioContent[10];
    int ioContentCount;
    bool isPermanent;       // True = PERM, False = TEMP
} MessageConfig;

MessageConfig permMessageConfigs[MAX_MESSAGE_CONFIGS];
MessageConfig tempMessageConfigs[MAX_MESSAGE_CONFIGS];

// Hlavní konfigurační struktura pro celý simulátor
typedef struct {
    char protocol[4];         // "104" nebo "101"
    char role[8];             // "SERVER"/"CLIENT"
    char ip[64];              // IP adresa (104)
    int port;                 // TCP port (104)
    char interface[64];       // Název rozhraní (101)
    int bandwidth;            // Rychlost (101)
    int originatorAddress;    // OA (většinou server)
    int commonAddress;        // CA (adresa stanice)
    int dataLogs;             // 1=datové logy zapnuty
    char dataPath[128];       // Cesta k datovým logům
    int serviceLogs;          // 1=servisní logy zapnuty
    char servicePath[128];    // Cesta k servisním logům
    int period;               // Perioda odesílání zpráv v sekundách
    int spontaneousEnable;    // 1=spontánní zapnuty (jen server)
    int spontaneousMin;       // Min. interval spontánních zpráv
    int spontaneousMax;       // Max. interval spontánních zpráv
    int multiplier;           // Kolikrát poslat každou zprávu
    int sync;                 // 1=klient posílá SYNC zprávy
    int disconnectAfterSend;  // 1=klient se odpojí po odeslání
} Config;

// =======================
// PROMĚNNÉ PRO ZPRÁVY A STAV
// =======================

static MessageConfig messageConfigs[MAX_MESSAGES];  // Pole konfigurací zpráv
static int numMessageConfigs = 0;                  // Počet načtených zpráv
bool isPermanent; // true = PERM_MESS, false = TEMP_MESS
static bool running = true;            // Hlavní smyčka běží/neběží
static time_t lastSentTime = 0;        // Poslední čas odeslání zprávy

// Spontánní zprávy (jen pro server)
static bool spontaneousEnabled = false;
static int minSpontaneousInterval = 2;
static int maxSpontaneousInterval = 10;
static time_t nextSpontaneousTime = 0;

static int multiplier = 1;                 // Multiplikátor zpráv
static int originatorAddress;              // OA z konfigu
static int commonAddress;                  // CA z konfigu
static CS104_Connection con = NULL;

bool allowMessages = false;                // Povoluje interaktivní zadávání zpráv
volatile sig_atomic_t configInterrupted = 0;   // Signalizace přerušení konfigurace
static jmp_buf configJump;                 // Pro návrat při přerušení konfigurace

// Prototypy pro funkce tisknoucí časové struktury
void printCP24Time2a(CP24Time2a time);
void printCP56Time2a(CP56Time2a time);

// =======================
// SIGNAL HANDLERY
// =======================

// Handler pro SIGINT (Ctrl+C) pro ukončení hlavní smyčky
void sigint_handler(int signalId) {
    running = false;
    if (con != NULL) {
        CS104_Connection_close(con);
        CS104_Connection_destroy(con);
        con = NULL;
    }
    printf("\nProgram ukončen pomocí Ctrl+C (SIGINT). Spojení uzavřeno.\n");
}


// Handler pro přerušení během interaktivní konfigurace
void sigintHandler_config(int sig) {
    printf("\nKonfigurace přerušena (Ctrl+C). Změny nebyly uloženy.\n");
    longjmp(configJump, 1);
}

const char* getCOTName(int cot) {
    switch (cot) {
        case 1: return "per/cyc";
        case 2: return "background";
        case 3: return "spontaneous";
        case 4: return "initialized";
        case 5: return "request";
        case 6: return "activation";
        case 7: return "confirmation";
        case 8: return "deactivation";
        case 9: return "activation confirm";
        case 10: return "deactivation confirm";
        case 20: return "interrogation";
        case 21: return "counter interrogation";
        case 44: return "read command";
        case 45: return "write command";
        default: return "unknown";
    }
}


// =======================
// BLOK: LOGOVACÍ FUNKCE (pro server i klienta)
// =======================

// Zaloguje spuštění instance (Client/Server)
void LogSTART(const char *role) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d %s started \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, role);
    fclose(fp);
}

// Log přijetí požadavku na spojení (server i klient)
void LogCONREQ(const char *ipAddress) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d New connection request from %s\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ipAddress);
    fclose(fp);
}

// Log otevření spojení (SERVER)/navázání spojení (CLIENT)
void LogCONOPEN() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection opened\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

// Log ukončení spojení
void LogCONCLOSED() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection closed\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

// (Používá se na serveru pro stav spojení)
void LogCONACT() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection activated\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}
void LogCONDEACT() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection deactivated\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

// Pouze klient (navázání TCP spojení)
void LogCONEST() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection established\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

// Přijetí řídících zpráv (STARTDT/STOPDT) na klientovi
void LogCONSTARTTD() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received STARTDT_CON\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}
void LogCONSTOPTD() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received STOPDT_CON\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}





/*void LogCONREF() { // (Nepoužíváno, možno odstranit)
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connections refreshed\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}*/



// =======================
// DATOVÉ LOGY – společné pro server i klienta
// =======================

// Log odeslané zprávy (datová vrstva)
void LogTX(int type, int elements, int oa, int ca) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d oa: %i ca: %i type:(%i) elements: %i\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, oa, ca, type, elements);
    fclose(fp);
}

// Log přijaté zprávy (datová vrstva)
void LogRX(int type, int elements, int oa, int ca) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d oa: %i ca: %i type:(%i) elements: %i\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, oa, ca, type, elements);
    fclose(fp);
}

// Log hodnoty (odeslané/přijaté) bez časové známky
void LogTXwoT(int ioa, float value) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value);
    fclose(fp);
}
void LogRXwoT(int ioa, float value) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value);
    fclose(fp);
}

// Log s časovou známkou (CP56Time2a)
void LogTXwT(int ioa, float value, CP56Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %04i-%02i-%02i %02i:%02i:%02i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value,
            CP56Time2a_getYear(timestamp) + 2000, CP56Time2a_getMonth(timestamp), CP56Time2a_getDayOfMonth(timestamp),
            CP56Time2a_getHour(timestamp), CP56Time2a_getMinute(timestamp), CP56Time2a_getSecond(timestamp));
    fclose(fp);
}

void LogTXwT24(int ioa, float value, CP24Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %02i:%02i:%03i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value,
            CP24Time2a_getMinute(timestamp), CP24Time2a_getSecond(timestamp), CP24Time2a_getMillisecond(timestamp));
    fclose(fp);
}

void LogRXwT(int ioa, float value, CP56Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %04i-%02i-%02i %02i:%02i:%02i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value,
            CP56Time2a_getYear(timestamp) + 2000, CP56Time2a_getMonth(timestamp), CP56Time2a_getDayOfMonth(timestamp),
            CP56Time2a_getHour(timestamp), CP56Time2a_getMinute(timestamp), CP56Time2a_getSecond(timestamp));
    fclose(fp);
}

void LogRXwT24(int ioa, float value, CP24Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %02i:%02i:%03i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ioa, value,
            CP24Time2a_getMinute(timestamp), CP24Time2a_getSecond(timestamp), CP24Time2a_getMillisecond(timestamp));
    fclose(fp);
}

// Log odeslané/přijaté zprávy typu "interrogation" (QOI)
void LogRXrequest(int qoi) { // server přijme dotaz
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received interrogation for group (%i).\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, qoi);
    fclose(fp);
}
void LogTXrequest(int qoi) { // klient posílá interrogation
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Transceived interrogation for group (%i).\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, qoi);
    fclose(fp);
}

// =======================
// FUNKCE PRO ČASOVÉ TISKY (tisk CP24, CP56 do konzole)
// =======================

void printCP56Time2a(CP56Time2a time) {
    // Tisk ve formátu HH:MM:SS DD/MM/YYYY
    printf("%02i:%02i:%02i %02i/%02i/%04i",
           CP56Time2a_getHour(time),
           CP56Time2a_getMinute(time),
           CP56Time2a_getSecond(time),
           CP56Time2a_getDayOfMonth(time),
           CP56Time2a_getMonth(time),
           CP56Time2a_getYear(time) + 2000);
}

void printCP24Time2a(CP24Time2a time) {
    // Tisk pouze minuty a sekundy
    printf("%02imin %02isec",
           CP24Time2a_getMinute(time),
           CP24Time2a_getSecond(time));
}

// =======================
// ČTENÍ HODNOT Z KONFIGURAČNÍHO SOUBORU (funkce na čtení jednoho klíče)
// =======================

char *readConfigValue(const char *filePath, const char *key) {
    FILE *file = fopen(filePath, "r");
    if (file == NULL) {
        perror("Failed to open file");
        return NULL;
    }
    char *value = NULL;
    char line[1024];

    while (fgets(line, sizeof(line), file)) {
        char *equalPos = strchr(line, '=');
        if (equalPos) {
            char keyInFile[128];
            strncpy(keyInFile, line, equalPos - line);
            keyInFile[equalPos - line] = '\0';

            // Ořízni mezery vlevo
            char *trimmedKey = keyInFile;
            while (*trimmedKey == ' ') trimmedKey++;

            if (strcmp(trimmedKey, key) == 0) {
                char *startOfValue = equalPos + 1;
                while (*startOfValue == ' ') startOfValue++;
                char *endOfValue = startOfValue + strcspn(startOfValue, "\r\n");
                *endOfValue = '\0';
                value = strdup(startOfValue);
                break;
            }
        }
    }
    fclose(file);
    return value;
}

// =======================
// FUNKCE PRO NAČTENÍ CELÉ KONFIGURACE
// =======================

// Načte všechny hodnoty z konfiguračního souboru do struktury Config
Config readFullConfig(const char* path) {
    Config cfg;
    memset(&cfg, 0, sizeof(Config));

    char* val;

    val = readConfigValue(path, "PROTOCOL");
    if (val) { strncpy(cfg.protocol, val, sizeof(cfg.protocol)); free(val); }
    val = readConfigValue(path, "ROLE");
    if (val) { strncpy(cfg.role, val, sizeof(cfg.role)); free(val); }
    val = readConfigValue(path, "IP");
    if (val) { strncpy(cfg.ip, val, sizeof(cfg.ip)); free(val); }
    val = readConfigValue(path, "PORT");
    if (val) { cfg.port = atoi(val); free(val); }
    val = readConfigValue(path, "INTERFACE");
    if (val) { strncpy(cfg.interface, val, sizeof(cfg.interface)); free(val); }
    val = readConfigValue(path, "BANDWIDTH");
    if (val) { cfg.bandwidth = atoi(val); free(val); }
    val = readConfigValue(path, "ORIGINATOR_ADDRESS");
    if (val) { cfg.originatorAddress = atoi(val); free(val); }
    val = readConfigValue(path, "COMMON_ADDRESS");
    if (val) { cfg.commonAddress = atoi(val); free(val); }
    val = readConfigValue(path, "DATALOGS");
    if (val) { cfg.dataLogs = atoi(val); free(val); }
    val = readConfigValue(path, "DATAPATH");
    if (val) { strncpy(cfg.dataPath, val, sizeof(cfg.dataPath)); free(val); }
    val = readConfigValue(path, "SERVICELOGS");
    if (val) { cfg.serviceLogs = atoi(val); free(val); }
    val = readConfigValue(path, "SERVICEPATH");
    if (val) { strncpy(cfg.servicePath, val, sizeof(cfg.servicePath)); free(val); }
    val = readConfigValue(path, "PERIOD");
    if (val) { cfg.period = atoi(val); free(val); }
    val = readConfigValue(path, "SPONTANEOUS");
    if (val) {
        sscanf(val, "%d;%d;%d", &cfg.spontaneousEnable, &cfg.spontaneousMin, &cfg.spontaneousMax);
        free(val);
    }
    val = readConfigValue(path, "MULTI");
    if (val) { cfg.multiplier = atoi(val); free(val); }
    val = readConfigValue(path, "SYNC");
    if (val) { cfg.sync = atoi(val); free(val); }
    val = readConfigValue(path, "DISCONNECTAFTERSEND");
    if (val) { cfg.disconnectAfterSend = atoi(val); free(val); }

    return cfg;
}

// =======================
// ČASOVÉ UTILITY PRO CP24
// =======================

// Nastaví čas do struktury CP24Time2a podle ms timestampu
void CP24Time2a_setFromMsTimestamp(CP24Time2a self, uint64_t timestamp) {
    memset(self->encodedValue, 0, 7);
    time_t timeVal = timestamp / 1000;
    int msPart = timestamp % 1000;
    struct tm tmTime;
    gmtime_r(&timeVal, &tmTime);

    CP24Time2a_setMillisecond(self, msPart);
    CP24Time2a_setSecond(self, tmTime.tm_sec);
    CP24Time2a_setMinute(self, tmTime.tm_min);
}

// Vytvoří CP24Time2a ze zadaného timestampu (nebo alokuje nový)
CP24Time2a CP24Time2a_createFromMsTimestamp(CP24Time2a self, uint64_t msTimestamp) {
    if (self == NULL)
        self = (CP24Time2a) GLOBAL_CALLOC(1, sizeof(struct sCP24Time2a));
    else
        memset(self, 0, sizeof(struct sCP24Time2a));
    if (self != NULL)
        CP24Time2a_setFromMsTimestamp(self, msTimestamp);
    return self;
}

// =======================
// VYTVOŘENÍ INFORMACE (InformationObject) DLE TYPU PRO SERVER
// =======================

InformationObject createIO(int messageType, int ioa, float value) {
    InformationObject io = NULL;
    switch (messageType) {
        case 1: // Single Point Information
        {
            bool valbool = (value != 0.0);
            io = (InformationObject) SinglePointInformation_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD);
            break;
        }
        case 2: // Single Point w/ CP24Time2a
        {
            bool valbool = (value != 0.0);
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) SinglePointWithCP24Time2a_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 3: // Double Point Information
        {
            int valint = (int) value;
            io = (InformationObject) DoublePointInformation_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD);
            break;
        }
        case 4: // Double Point w/ CP24Time2a
        {
            int valint = (int) value;
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) DoublePointWithCP24Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 5: { // M_ST_NA_1 Step Position
            int val = (int)value;                // -64..+63
            bool isTransient = false;            // volitelně viz parsing níže
            io = (InformationObject) StepPositionInformation_create(NULL, ioa, val, isTransient, IEC60870_QUALITY_GOOD);
            break;
        }
        case 6: { // M_ST_TA_1 Step Position + CP24
            int val = (int)value;
            bool isTransient = false;
            CP24Time2a ts = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) StepPositionWithCP24Time2a_create(NULL, ioa, val, isTransient, IEC60870_QUALITY_GOOD, ts);
            break;
        }
        case 7: { // M_BO_NA_1 Bitstring32
            uint32_t v = (uint32_t)((int)value);
            io = (InformationObject) BitString32_createEx(NULL, ioa, v, IEC60870_QUALITY_GOOD);
            break;
        }
        case 8: { // M_BO_TA_1 Bitstring32 + CP24
            uint32_t v = (uint32_t)((int)value);
            io = (InformationObject) Bitstring32WithCP24Time2a_createEx(NULL, ioa, v, IEC60870_QUALITY_GOOD,
                                                                        CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 9: // Measured Normalized
        {
            io = (InformationObject) MeasuredValueNormalized_create(NULL, ioa, value, IEC60870_QUALITY_GOOD);
            break;
        }
        case 10: // Measured Normalized w/ CP24Time2a
        {
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueNormalizedWithCP24Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 11: // Measured Scaled
        {
            int valint = (int) value;
            io = (InformationObject) MeasuredValueScaled_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD);
            break;
        }
        case 12: // Measured Scaled w/ CP24Time2a
        {
            int valint = (int) value;
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueScaledWithCP24Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 13: // Measured Short Float
        {
            io = (InformationObject) MeasuredValueShort_create(NULL, ioa, value, IEC60870_QUALITY_GOOD);
            break;
        }
        case 14: // Measured Short w/ CP24Time2a
        {
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueShortWithCP24Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 15: { // M_IT_NA_1 Integrated totals (BCR)
            BinaryCounterReading bcr = GLOBAL_CALLOC(1, sizeof(struct sBinaryCounterReading));
            BinaryCounterReading_setValue(bcr, (int32_t)value);
            // volitelně: BinaryCounterReading_setSequenceNumber(bcr, 0); BinaryCounterReading_setCarry(bcr, false);
            io = (InformationObject) IntegratedTotals_create(NULL, ioa, bcr);
            break;
        }
        case 16: { // M_IT_TA_1 BCR + CP24
            BinaryCounterReading bcr = GLOBAL_CALLOC(1, sizeof(struct sBinaryCounterReading));
            BinaryCounterReading_setValue(bcr, (int32_t)value);
            io = (InformationObject) IntegratedTotalsWithCP24Time2a_create(NULL, ioa, bcr,
                                                                           CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 30: // Single Point w/ CP56Time2a
        {
            bool valbool = (value != 0.0);
            io = (InformationObject) SinglePointWithCP56Time2a_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD,
                                                                      CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 31: // Double Point w/ CP56Time2a
        {
            int valint = (int) value;
            io = (InformationObject) DoublePointWithCP56Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                      CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 32: { // M_ST_TB_1 Step Position + CP56
            int val = (int)value;
            bool isTransient = false;
            io = (InformationObject) StepPositionWithCP56Time2a_create(NULL, ioa, val, isTransient, IEC60870_QUALITY_GOOD,
                                                                       CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 33: { // M_BO_TB_1 Bitstring32 + CP56
            uint32_t v = (uint32_t)((int)value);
            io = (InformationObject) Bitstring32WithCP56Time2a_createEx(NULL, ioa, v, IEC60870_QUALITY_GOOD,
                                                                        CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 34: // Measured Normalized w/ CP56Time2a
        {
            io = (InformationObject) MeasuredValueNormalizedWithCP56Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD,
                                                                                  CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 35: // Measured Scaled w/ CP56Time2a
        {
            int valint = (int) value;
            io = (InformationObject) MeasuredValueScaledWithCP56Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                              CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 36: // Measured Short w/ CP56Time2a
        {
            io = (InformationObject) MeasuredValueShortWithCP56Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD,
                                                                             CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
        case 37: { // M_IT_TB_1 BCR + CP56
            BinaryCounterReading bcr = GLOBAL_CALLOC(1, sizeof(struct sBinaryCounterReading));
            BinaryCounterReading_setValue(bcr, (int32_t)value);
            io = (InformationObject) IntegratedTotalsWithCP56Time2a_create(NULL, ioa, bcr,
                                                                           CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        }
    }
    return io;
}

// =======================
// FUNKCE PRO NAČTENÍ KONFIGURACE ZPRÁV (iec_config.txt)
// =======================

// Načte konfiguraci zpráv do globálního pole messageConfigs[]
void readMessageConfig(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) { perror("Failed to open configuration file"); return; }
    char line[256];
    int currentMessageType = -1;
    bool currentPermanentFlag = false;
    numMessageConfigs = 0;

    while (fgets(line, sizeof(line), file) != NULL) {
        // Odstraň nový řádek
        line[strcspn(line, "\r\n")] = '\0';

        int messageType, ioa;
        float value1, value2;

        if (strcmp(line, "PERM_MESS=") == 0) { currentPermanentFlag = true; continue; }
        if (strcmp(line, "TEMP_MESS=") == 0) { currentPermanentFlag = false; continue; }
        if (strlen(line) == 0 || strchr(line, '=') != NULL) continue;

        // Nejprve dualní: typ;ioa;val1;val2
        if (sscanf(line, "%d;%d;%f;%f", &messageType, &ioa, &value1, &value2) == 4) {
            messageConfigs[numMessageConfigs].messageType = messageType;
            messageConfigs[numMessageConfigs].ioContent[0].ioa = ioa;
            messageConfigs[numMessageConfigs].ioContent[0].toggleValueA = value1;
            messageConfigs[numMessageConfigs].ioContent[0].toggleValueB = value2;
            messageConfigs[numMessageConfigs].ioContent[0].toggleEnabled = true;
            messageConfigs[numMessageConfigs].ioContent[0].toggleState = false;
            messageConfigs[numMessageConfigs].ioContentCount = 1;
            messageConfigs[numMessageConfigs].isPermanent = currentPermanentFlag;
            numMessageConfigs++;
            continue;
        }

        // Pak statické: typ;ioa;val
        if (sscanf(line, "%d;%d;%f", &messageType, &ioa, &value1) == 3) {
            messageConfigs[numMessageConfigs].messageType = messageType;
            messageConfigs[numMessageConfigs].ioContent[0].ioa = ioa;
            messageConfigs[numMessageConfigs].ioContent[0].value = value1;
            messageConfigs[numMessageConfigs].ioContent[0].toggleEnabled = false;
            messageConfigs[numMessageConfigs].ioContentCount = 1;
            messageConfigs[numMessageConfigs].isPermanent = currentPermanentFlag;
            numMessageConfigs++;
            continue;
        }
    }
    fclose(file);
}

// Handler pro odeslaný ASDU – vypíše, zaloguje, zpracuje IO podle typu
static bool asduTransmitHandler(CS101_ASDU asdu) {
    printf("TRANSMITTED ASDU - OA: %i CA: %i TYPE: %s(%i) NUMBER OF IOs: %i \n",
           CS101_ASDU_getOA(asdu),
           CS101_ASDU_getCA(asdu),
           TypeID_toString(CS101_ASDU_getTypeID(asdu)),
           CS101_ASDU_getTypeID(asdu),
           CS101_ASDU_getNumberOfElements(asdu));

    if (dataConfig == 1) { // Filtration of general messages
        LogTX(CS101_ASDU_getTypeID(asdu), CS101_ASDU_getNumberOfElements(asdu), CS101_ASDU_getOA(asdu),
              CS101_ASDU_getCA(asdu));
    }

    switch (CS101_ASDU_getTypeID(asdu)) {
        case M_SP_NA_1: {
            printf("  single point information:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointInformation io = (SinglePointInformation) CS101_ASDU_getElement(asdu, i);
                if (io == NULL) {
                    printf("Error: Failed to retrieve information object at index %d.\n", i);
                    continue;
                }

                int value = SinglePointInformation_getValue(io);
                bool booleanValue = (value != 0); // Assuming 0 is False, any non-zero is True

                // Display the Information Object Address and the value in a descriptive format
                printf("    IOA: %i value: %s\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       booleanValue ? "True" : "False");

                // Conditionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             value); // Log the original integer value
                }

                SinglePointInformation_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_SP_TA_1: {
            printf("  single point information with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointWithCP24Time2a io = (SinglePointWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                bool value = SinglePointInformation_getValue(io); // Gets the boolean state

                // Print the IOA, value as True/False, and the formatted timestamp
                printf("    IOA: %i value: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value ? "True" : "False");

                // Function to print the timestamp in a readable format
                printCP24Time2a(SinglePointWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwT24(InformationObject_getObjectAddress((InformationObject) io),
                              value, // Logging the actual boolean as 1 (True) or 0 (False)
                              SinglePointWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup after use
                SinglePointWithCP24Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_DP_NA_1: {
            printf("  double point information:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointInformation io = (DoublePointInformation) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue(io);

                const char *valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA and the descriptive value
                printf("    IOA: %i value: %s\n", InformationObject_getObjectAddress((InformationObject) io),
                       valueDescription);

                // Conditionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value; // If your LogRXwoT expects a float, convert int to float here
                    LogTXwoT(InformationObject_getObjectAddress((InformationObject) io), logValue);
                }

                DoublePointInformation_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_DP_TA_1: {
            printf("  double point information with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointWithCP24Time2a io = (DoublePointWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue((DoublePointInformation) io);

                const char *valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA, value description and timestamp
                printf("    IOA: %i value: %s time: ", InformationObject_getObjectAddress((InformationObject) io),
                       valueDescription);

                // Function to print time in a readable format
                printCP24Time2a(DoublePointWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value;
                    LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), value,
                              DoublePointWithCP24Time2a_getTimestamp(io));
                }

                DoublePointWithCP24Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_NA_1: {
            printf("  measured value, normalized value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueNormalized io = (MeasuredValueNormalized) CS101_ASDU_getElement(asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);

                // Display detailed information
                printf("    IOA: %i, Normalized Value: %6.3f\n",
                       InformationObject_getObjectAddress((InformationObject) io), normalizedValue);

                // Optionally log the data if enabled
                if (dataConfig == 1) {
                    LogTXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             normalizedValue);
                }

                // Cleanup to avoid memory leaks
                MeasuredValueNormalized_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TA_1: {
            printf("  measured normalized value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueNormalizedWithCP24Time2a io =
                        (MeasuredValueNormalizedWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);
                CP24Time2a timestamp = MeasuredValueNormalizedWithCP24Time2a_getTimestamp(io);

                // Display detailed information including time stamp
                printf("    IOA: %i, Normalized Value: %6.3f, Timestamp: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       normalizedValue);
                printCP24Time2a(timestamp);

                // Optionally log the data if enabled
                if (dataConfig == 1) {
                    LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), normalizedValue,
                              MeasuredValueNormalizedWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup to avoid memory leaks
                MeasuredValueNormalizedWithCP24Time2a_destroy(io);
                printf("\n"); // Ensure each entry is on a new line
            }
            break;
        }
        case M_ME_NB_1: {
            printf("  measured value, scaled value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaled io =
                        (MeasuredValueScaled) CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueScaled_getValue(io); // Extract value

                // Print IOA and value with better formatting
                printf("    IOA: %i, Scaled Value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Optionally log the data if data logging is enabled
                if (dataConfig == 1) {
                    LogTXwoT(InformationObject_getObjectAddress((InformationObject) io), value);
                }

                // Cleanup to avoid memory leaks
                MeasuredValueScaled_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TB_1: {
            printf("  measured value, scaled value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaled io =
                        (MeasuredValueScaled) CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueScaled_getValue(io); // Get the scaled value

                // Print IOA, value and timestamp in a structured way
                printf("    IOA: %i, value: %d, time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);
                printCP24Time2a(MeasuredValueScaledWithCP24Time2a_getTimestamp(io)); // Output the timestamp

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwT24(InformationObject_getObjectAddress((InformationObject) io),
                              value,
                              MeasuredValueScaledWithCP24Time2a_getTimestamp(io)); // Log with timestamp
                }

                // Cleanup after use
                MeasuredValueScaledWithCP24Time2a_destroy(io);
                printf("\n"); // Ensure new line for next data point
            }
            break;
        }
        case M_ME_NC_1: {
            printf("  measured value, short value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShort io = (MeasuredValueShort) CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueShort_getValue(io); // Get the short value

                // Print the IOA and value with improved formatting for clarity
                printf("    IOA: %i, value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             value); // Log the actual value
                }

                // Cleanup after use
                MeasuredValueShort_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }

        case M_ME_TC_1: { //TADY TOE DOBRY MYSLIM
            printf("  measured value, short value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShortWithCP24Time2a io = (MeasuredValueShortWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueShort_getValue(io); // Get the short value

                // Print the IOA, value, and format the timestamp
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Function to print the timestamp in a readable format
                printCP24Time2a(MeasuredValueShortWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), value,
                              MeasuredValueShortWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup after use
                MeasuredValueShortWithCP24Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_SP_TB_1: {
            printf("  single point information with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointWithCP56Time2a io = (SinglePointWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                bool value = SinglePointInformation_getValue(io); // Gets the boolean state

                // Print the IOA, value as True/False, and the formatted timestamp
                printf("    IOA: %i value: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value ? "True" : "False");

                // Function to print the timestamp in a readable format
                printCP56Time2a(SinglePointWithCP56Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogTXwT(InformationObject_getObjectAddress((InformationObject) io), value,
                            SinglePointWithCP56Time2a_getTimestamp(io));
                }

                // Cleanup after use
                SinglePointWithCP56Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }

        case M_DP_TB_1: {
            printf("  double point information with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointWithCP56Time2a io = (DoublePointWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue((DoublePointInformation) io);

                const char *valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA, descriptive value and timestamp
                printf("    IOA: %i value: %s time: ", InformationObject_getObjectAddress((InformationObject) io),
                       valueDescription);

                // Function to print time in a readable format
                printCP56Time2a(DoublePointWithCP56Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value;
                    LogTXwT(InformationObject_getObjectAddress((InformationObject) io), value,
                            DoublePointWithCP56Time2a_getTimestamp(io));
                }

                DoublePointWithCP56Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TD_1: {
            int i;

            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueNormalizedWithCP56Time2a io = (MeasuredValueNormalizedWithCP56Time2a) CS101_ASDU_getElement(
                        asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);
                //InformationObject_getObjectAddress((InformationObject) io), normalizedValue;
                printCP56Time2a(MeasuredValueNormalizedWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogTXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueNormalized_getValue((MeasuredValueNormalized) io),
                            MeasuredValueNormalizedWithCP56Time2a_getTimestamp(io));
                }

                MeasuredValueNormalizedWithCP56Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is separated clearly
            }
            break;
        }
        case M_ME_TE_1: {
            printf("  measured scaled value with CP56Time2a timestamp:\n");

            int i;

            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaledWithCP56Time2a io = (MeasuredValueScaledWithCP56Time2a) CS101_ASDU_getElement(asdu,
                                                                                                                 i);
                int value = MeasuredValueScaled_getValue(io); // Extract value
                // Assuming the values are scaled and should be displayed as floating-point for better precision
                printf("    IOA: %i, value: %d, time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);
                printCP56Time2a(MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogTXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueScaled_getValue((MeasuredValueScaled) io),
                            MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                }
                MeasuredValueScaledWithCP56Time2a_destroy(io);
                printf("\n"); // Adding a newline for better readability between entries
            }
            break;
        }
        case M_ME_TF_1: {
            printf("  measured short float value with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShortWithCP56Time2a io = (MeasuredValueShortWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                printf("    IOA: %i value: %.2f time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       MeasuredValueShort_getValue((MeasuredValueShort) io));
                printCP56Time2a(MeasuredValueShortWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogTXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueShort_getValue((MeasuredValueShort) io),
                            MeasuredValueShortWithCP56Time2a_getTimestamp(io));
                }
                MeasuredValueShortWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
            /* === NEW: Step position (5,6,32) === */
        case M_ST_NA_1: { // 5
            printf("  step position information:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionInformation io = (StepPositionInformation) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue(io);            // -64..+63
                bool tr = StepPositionInformation_isTransient(io);
                printf("    IOA: %i value: %d transient: %s\n",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                if (dataConfig == 1) LogTXwoT(InformationObject_getObjectAddress((InformationObject) io), v);
                StepPositionInformation_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_ST_TA_1: { // 6 + CP24
            printf("  step position information with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionWithCP24Time2a io = (StepPositionWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue((StepPositionInformation) io);
                bool tr = StepPositionInformation_isTransient((StepPositionInformation) io);
                printf("    IOA: %i value: %d transient: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                printCP24Time2a(StepPositionWithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), v,
                                               StepPositionWithCP24Time2a_getTimestamp(io));
                StepPositionWithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_ST_TB_1: { // 32 + CP56
            printf("  step position information with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionWithCP56Time2a io = (StepPositionWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue((StepPositionInformation) io);
                bool tr = StepPositionInformation_isTransient((StepPositionInformation) io);
                printf("    IOA: %i value: %d transient: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                printCP56Time2a(StepPositionWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT(InformationObject_getObjectAddress((InformationObject) io), v,
                                             StepPositionWithCP56Time2a_getTimestamp(io));
                StepPositionWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

            /* === NEW: Bitstring32 (7,8,33) === */
        case M_BO_NA_1: { // 7
            printf("  bitstring32 value:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                BitString32 io = (BitString32) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue(io);
                printf("    IOA: %i value: 0x%08x\n",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                if (dataConfig == 1) LogTXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)v);
                BitString32_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_BO_TA_1: { // 8 + CP24
            printf("  bitstring32 value with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                Bitstring32WithCP24Time2a io = (Bitstring32WithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue((BitString32) io);
                printf("    IOA: %i value: 0x%08x time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                printCP24Time2a(Bitstring32WithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), (float)v,
                                               Bitstring32WithCP24Time2a_getTimestamp(io));
                Bitstring32WithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_BO_TB_1: { // 33 + CP56
            printf("  bitstring32 value with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                Bitstring32WithCP56Time2a io = (Bitstring32WithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue((BitString32) io);
                printf("    IOA: %i value: 0x%08x time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                printCP56Time2a(Bitstring32WithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT(InformationObject_getObjectAddress((InformationObject) io), (float)v,
                                             Bitstring32WithCP56Time2a_getTimestamp(io));
                Bitstring32WithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

            /* === NEW: Integrated totals / BCR (15,16,37) === */
        case M_IT_NA_1: { // 15
            printf("  integrated totals (BCR):\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotals io = (IntegratedTotals) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR(io);
                int32_t val = BinaryCounterReading_getValue(b);
                /* NOTE: některé verze knihovny nemají getCarry/getAdjusted -> nevolat */
                /* int seq = BinaryCounterReading_getSequenceNumber(b);  // volitelné, pokud chceš */
                printf("    IOA: %i value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                if (dataConfig == 1) LogTXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)val);
                IntegratedTotals_destroy(io);
                printf("\n");
            }
            break;
        }

        case M_IT_TA_1: { // 16 + CP24
            printf("  integrated totals (BCR) with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotalsWithCP24Time2a io = (IntegratedTotalsWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR((IntegratedTotals) io);
                int32_t val = BinaryCounterReading_getValue(b);
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                printCP24Time2a(IntegratedTotalsWithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT24(InformationObject_getObjectAddress((InformationObject) io), (float)val,
                                               IntegratedTotalsWithCP24Time2a_getTimestamp(io));
                IntegratedTotalsWithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

        case M_IT_TB_1: { // 37 + CP56
            printf("  integrated totals (BCR) with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotalsWithCP56Time2a io = (IntegratedTotalsWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR((IntegratedTotals) io);
                int32_t val = BinaryCounterReading_getValue(b);
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                printCP56Time2a(IntegratedTotalsWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogTXwT(InformationObject_getObjectAddress((InformationObject) io), (float)val,
                                             IntegratedTotalsWithCP56Time2a_getTimestamp(io));
                IntegratedTotalsWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }


    }
    return true;
}



// Odešle spontánní zprávu (náhodně z vybraných typů pro spontánní)
void sendSpontaneousMessage104(CS104_Slave slave, CS101_AppLayerParameters alparams, int multiplier) {
    CS101_ASDU newAsdu = CS101_ASDU_create(alparams, true, CS101_COT_SPONTANEOUS, originatorAddress, commonAddress,
                                           false, false);
    InformationObject ios[MAX_MESSAGES];
    int numSpontIos = 0;
    // Projdi konfiguraci, najdi správné typy pro spontánní zprávy
    for (int i = 0; i < numMessageConfigs; ++i) {
        if (messageConfigs[i].messageType != 30 &&
            messageConfigs[i].messageType != 31 &&
            messageConfigs[i].messageType != 34 &&
            messageConfigs[i].messageType != 35 &&
            messageConfigs[i].messageType != 36) {
            continue;
        }
        MessageConfig msg = messageConfigs[i];
        for (int j = 0; j < msg.ioContentCount; ++j) {
            IOContent ioContent = msg.ioContent[j];
            ios[numSpontIos] = createIO(msg.messageType, ioContent.ioa, ioContent.value);
            ++numSpontIos;
        }
    }
    InformationObject io;
    // Vyber náhodně jednu IO nebo použij defaultní, pokud nejsou žádné
    if (numSpontIos != 0) {
        int index = rand() % (numSpontIos);
        io = ios[index];
    } else {
        io = (InformationObject) SinglePointWithCP56Time2a_create(NULL, 9999, 1, IEC60870_QUALITY_GOOD,
                                                                  CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
    }
    CS101_ASDU_addInformationObject(newAsdu, io);
    for (int i = 0; i < multiplier; ++i) {
        CS104_Slave_enqueueASDU(slave, newAsdu);
        asduTransmitHandler(newAsdu);
    }
    if (numSpontIos == 0) {
        InformationObject_destroy(io);
    }
    CS101_ASDU_destroy(newAsdu);
}

// Odešle spontánní zprávu (náhodně z vybraných typů pro spontánní)
void sendSpontaneousMessage101(CS104_Slave slave, CS101_AppLayerParameters alparams, int multiplier) {
    CS101_ASDU newAsdu = CS101_ASDU_create(alparams, true, CS101_COT_SPONTANEOUS, originatorAddress, commonAddress,
                                           false, false);
    InformationObject ios[MAX_MESSAGES];
    int numSpontIos = 0;
    // Projdi konfiguraci, najdi správné typy pro spontánní zprávy
    for (int i = 0; i < numMessageConfigs; ++i) {
        if (messageConfigs[i].messageType != 30 &&
            messageConfigs[i].messageType != 31 &&
            messageConfigs[i].messageType != 34 &&
            messageConfigs[i].messageType != 35 &&
            messageConfigs[i].messageType != 36) {
            continue;
        }
        MessageConfig msg = messageConfigs[i];
        for (int j = 0; j < msg.ioContentCount; ++j) {
            IOContent ioContent = msg.ioContent[j];
            ios[numSpontIos] = createIO(msg.messageType, ioContent.ioa, ioContent.value);
            ++numSpontIos;
        }
    }
    InformationObject io;
    // Vyber náhodně jednu IO nebo použij defaultní, pokud nejsou žádné
    if (numSpontIos != 0) {
        int index = rand() % (numSpontIos);
        io = ios[index];
    } else {
        io = (InformationObject) SinglePointWithCP56Time2a_create(NULL, 9999, 1, IEC60870_QUALITY_GOOD,
                                                                  CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
    }
    CS101_ASDU_addInformationObject(newAsdu, io);
    for (int i = 0; i < multiplier; ++i) {
        CS101_Slave_enqueueUserDataClass1(slave, newAsdu);
        asduTransmitHandler(newAsdu);
    }
    if (numSpontIos == 0) {
        InformationObject_destroy(io);
    }
    CS101_ASDU_destroy(newAsdu);
}


// Nastaví parametry spontánních zpráv z řetězce "1;min;max"
void configureSpontaneousMessages(const char *config) {
    char *configCopy = strdup(config);
    char *token = strtok(configCopy, ";");
    if (token && atoi(token) == 1) {
        spontaneousEnabled = true;
        token = strtok(NULL, ";");
        minSpontaneousInterval = atoi(token);
        token = strtok(NULL, ";");
        maxSpontaneousInterval = atoi(token);
    }
    free(configCopy);
}

// Určí náhodný čas další spontánní zprávy v intervalu <min, max>
void scheduleNextSpontaneousMessage() {
    if (!spontaneousEnabled) return;
    int interval = minSpontaneousInterval + rand() % (maxSpontaneousInterval - minSpontaneousInterval + 1);
    nextSpontaneousTime = time(NULL) + interval;
}




/* Handler pro logování surových zpráv (nepovinné, hlavně pro ladění) */
static void rawMessageHandler(void *parameter, IMasterConnection connection, uint8_t *msg, int msgSize, bool sent) {
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");
    for (int i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }
    printf("\n");
}


/* Handler pro příjem SYNC příkazu (synchronizace času od klienta) */
static bool clockSyncHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime) {
    printf("[SERVER] Process time sync command with UTC time ");
    printCP56Time2a(newTime);
    printf("\n");

    // V reálném nasazení bys zde aktualizoval čas zařízení (uint64t)
    uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);
    CP56Time2a_setFromMsTimestamp(newTime, Hal_getTimeInMs()); // Tady pouze přepíše čas zpět na aktuální HAL čas

    return true;
}

/* Handler pro přijetí interrogation příkazu (klient se ptá na data) */
static bool interrogationHandler(void *parameter, IMasterConnection connection, CS101_ASDU requestAsdu, uint8_t qoi) {
    printf("[SERVER] Received interrogation for group %i\n", qoi);

    if (qoi == 20) { // Odpovídáme pouze na QOI = 20 (typická volba)
        printf("[SERVER] Answering the interrogation command (QOI = 20)\n");
        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
        if (serviceConfig == 1) {
            LogRXrequest(qoi);
        }
        IMasterConnection_sendACT_CON(connection, requestAsdu, false);

        // Připravíme ASDU pro každý typ zprávy
        CS101_ASDU asdus[MAX_MESSAGE_TYPES] = {0};

        for (int i = 0; i < numMessageConfigs; i++) {
            MessageConfig msg = messageConfigs[i];
            if (msg.messageType < MAX_MESSAGE_TYPES) {
                if (!asdus[msg.messageType]) {
                    asdus[msg.messageType] = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                                                               originatorAddress, commonAddress, false, false);
                    if (asdus[msg.messageType] == NULL) {
                        fprintf(stderr, "Failed to create ASDU for type %d\n", msg.messageType);
                        continue;
                    }
                }
                // Přidáme všechny IO pro tento typ
                for (int j = 0; j < msg.ioContentCount; j++) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);

                    if (io != NULL) {
                        if (!CS101_ASDU_addInformationObject(asdus[msg.messageType], io)) {
                            printf("Failed to add IO (Type %d, IOA %d) to ASDU\n", msg.messageType, ioContent.ioa);
                        }
                        InformationObject_destroy(io);
                    } else {
                        printf("Failed to create IO (Type %d, IOA %d)\n", msg.messageType, ioContent.ioa);
                    }
                }
            } else {
                fprintf(stderr, "Message type %d out of bounds\n", msg.messageType);
            }
        }
        // Odeslání všech odpovědí
        for (int k = 0; k < MAX_MESSAGE_TYPES; k++) {
            if (asdus[k] && CS101_ASDU_getNumberOfElements(asdus[k]) > 0) {
                IMasterConnection_sendASDU(connection, asdus[k]);
                asduTransmitHandler(asdus[k]);
            }
            if (asdus[k]) {
                CS101_ASDU_destroy(asdus[k]);
            }
        }
    } else {
        // Na jiné QOI pouze pozitivně potvrdíme
        IMasterConnection_sendACT_CON(connection, requestAsdu, true);
    }
    return true;
}

// Handler pro přijatý ASDU na serveru – zpracovává příkazy 45/46
static bool asduHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu) {
    int type = CS101_ASDU_getTypeID(asdu);
    int cot = CS101_ASDU_getCOT(asdu);
    int oa = CS101_ASDU_getOA(asdu);
    int ca = CS101_ASDU_getCA(asdu);
    int numIO = CS101_ASDU_getNumberOfElements(asdu);

    printf("RECVD ASDU | OA: %d | CA: %d | TYPE: %s(%d) | COT: %d (%s) | IOs: %d\n",
           oa, ca, TypeID_toString(type), type, cot, getCOTName(cot), numIO);
    if (dataConfig == 1) {
        LogRX(CS101_ASDU_getTypeID(asdu), CS101_ASDU_getNumberOfElements(asdu),
              CS101_ASDU_getOA(asdu), CS101_ASDU_getCA(asdu));
    }

    switch (CS101_ASDU_getTypeID(asdu)) {
        case C_SC_NA_1: // 45 Single Command
            printf("  Single command:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SingleCommand io = (SingleCommand) CS101_ASDU_getElement(asdu, i);
                bool value = SingleCommand_getState(io);
                printf("    IOA: %i value: %s \n",
                       InformationObject_getObjectAddress((InformationObject) io), value ? "true" : "false");
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)value);
                }
                SingleCommand_destroy(io);
            }
            break;
        case C_DC_NA_1: // 46 Double Command
            printf("  Double command:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoubleCommand io = (DoubleCommand) CS101_ASDU_getElement(asdu, i);
                int value = DoubleCommand_getState(io);
                const char *valueDescription;
                switch (value) {
                    case 0: valueDescription = "NOT PERMITTED"; break;
                    case 1: valueDescription = "OFF"; break;
                    case 2: valueDescription = "ON"; break;
                    case 3: valueDescription = "NOT PERMITTED"; break;
                    default: valueDescription = "UNKNOWN"; break;
                }
                printf("    IOA: %i value: %s\n", InformationObject_getObjectAddress((InformationObject) io), valueDescription);
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)value);
                }
                DoubleCommand_destroy(io);
            }
            break;
    }
    return true;
}

// Handler událostí spojení pro klienta
static void connectionHandler(void *parameter, CS104_Connection connection, CS104_ConnectionEvent event) {
    switch (event) {
        case CS104_CONNECTION_OPENED:
            printf("[CLIENT - 104] Connection established\n");
            if (serviceConfig == 1) LogCONEST();
            break;
        case CS104_CONNECTION_CLOSED:
            printf("[CLIENT - 104] Connection closed\n");
            if (serviceConfig == 1) LogCONCLOSED();
            break;
        case CS104_CONNECTION_FAILED:
            printf("[CLIENT - 104] Failed to connect\n");
            break;
        case CS104_CONNECTION_STARTDT_CON_RECEIVED:
            printf("[CLIENT - 104] Received STARTDT_CON\n");
            if (serviceConfig == 1) LogCONSTARTTD();
            break;
        case CS104_CONNECTION_STOPDT_CON_RECEIVED:
            printf("[CLIENT - 104] Received STOPDT_CON\n");
            if (serviceConfig == 1) LogCONSTOPTD();
            break;
    }
}

static void linkLayerStateChanged(void* parameter, int address, LinkLayerState state)
{
    printf("[101] Link layer state: ");

    switch (state) {
        case LL_STATE_IDLE:
            printf("IDLE\n");
            break;
        case LL_STATE_ERROR:
            printf("ERROR\n");
            break;
        case LL_STATE_BUSY:
            printf("BUSY\n");
            break;
        case LL_STATE_AVAILABLE:
            printf("AVAILABLE\n");
            break;
    }

    if (state == LL_STATE_AVAILABLE) {
        printf("[101] Connected!\n");
    }
}


/*101*/
static void resetCUHandler(void* parameter)
{
    printf("Received reset CU\n");
    CS101_Slave_flushQueues((CS101_Slave) parameter);
}

// Handler pro žádost o spojení (pouze vypíše a povolí spojení)
static bool connectionRequestHandler(void *parameter, const char *ipAddress) {
    printf("[SERVER - 104] New connection request from %s\n", ipAddress);
    if (serviceConfig == 1) LogCONREQ(ipAddress);
    // Přijímáme všechny spojení (v budoucnu můžeš přidat podmínky)
    return true;
}

// Handler pro události na serveru (otevření, aktivace, zavření spojení apod.)
static void connectionEventHandler(void *parameter, IMasterConnection con, CS104_PeerConnectionEvent event) {
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("[SERVER - 104] Connection opened (%p)\n", con);
        if (serviceConfig == 1) LogCONOPEN();
    } else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("[SERVER - 104] Connection closed (%p)\n", con);
        if (serviceConfig == 1) LogCONCLOSED();
    } else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("[SERVER - 104] Connection activated (%p)\n", con);
        if (serviceConfig == 1) LogCONACT();
    } else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("[SERVER - 104] Connection deactivated (%p)\n", con);
        if (serviceConfig == 1) LogCONDEACT();
    }
}


void runInteractiveEditor(const char* configPath) {
    if (setjmp(configJump) != 0) {
        remove("iec_config.tmp");  // bezpečně smažeme dočasný soubor
        return;
    }

    signal(SIGINT, sigintHandler_config);  // aktivuj handler

    const char* tempPath = "iec_config.tmp";
    FILE* config = fopen(tempPath, "w");
    if (!config) {
        printf("Nelze otevřít dočasný soubor %s.\n", tempPath);
        return;
    }


    char input[256];
    char protocol[10], role[10];
    int allowMessages = 0;

    printf("=== INTERAKTIVNÍ KONFIGURAČNÍ PRŮVODCE ===\n");

    // --- Obecné ---
    printf("Zadej protokol simulátoru (1=IEC 101, 4=IEC 104): ");
    scanf("%s", input);
    if (strcmp(input, "1") == 0) strcpy(protocol, "101");
    else strcpy(protocol, "104");
    fprintf(config, "PROTOCOL=%s\n", protocol);

    printf("Zadej roli simulátoru (S=SERVER, C=CLIENT): ");
    scanf("%s", input);
    if (strcmp(input, "S") == 0) strcpy(role, "SERVER");
    else strcpy(role, "CLIENT");
    fprintf(config, "ROLE=%s\n\n", role);

    // --- Síťové rozhraní (pro 104) ---
    if (strcmp(protocol, "104") == 0) {
        printf("Zadej IP adresu serveru: ");
        scanf("%s", input);
        fprintf(config, "IP=%s\n", input);

        printf("Zadej port serveru: ");
        scanf("%s", input);
        fprintf(config, "PORT=%s\n\n", input);
    }

    // --- Sériové rozhraní (pro 101) ---
    if (strcmp(protocol, "101") == 0) {
        printf("Zadej název rozhraní (např. /dev/ttyS0) [Enter = výchozí]: ");
        getchar(); // remove newline from previous input
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0;  // remove newline

        if (strlen(input) == 0)
            strcpy(input, "/dev/ttyS0");
        fprintf(config, "INTERFACE=%s\n", input);

        printf("Zadej rychlost přenosu (např. 9600) [Enter = výchozí]: ");
        fgets(input, sizeof(input), stdin);
        input[strcspn(input, "\n")] = 0;

        if (strlen(input) == 0)
            strcpy(input, "9600");
        fprintf(config, "BANDWIDTH=%s\n\n", input);
    }


    // --- Adresy ---
    int oa = -1;
    do {
        printf("Zadej ORIGINATOR_ADDRESS (0-255): ");
        scanf("%d", &oa);
    } while (oa < 0 || oa > 255);
    fprintf(config, "ORIGINATOR_ADDRESS=%d\n", oa);

    int ca = -1;
    do {
        printf("Zadej COMMON_ADDRESS (0-65535): ");
        scanf("%d", &ca);
    } while (ca < 0 || ca > 65535);
    fprintf(config, "COMMON_ADDRESS=%d\n\n", ca);

    // --- Logování ---
    printf("Povolit DATALOGS? (1=ano, 0=ne): ");
    scanf("%s", input);
    fprintf(config, "DATALOGS=%s\n", input);
    if (strcmp(input, "1") == 0) {
        printf("Zadej cestu k DATALOG souboru: ");
        scanf("%s", input);
        fprintf(config, "DATAPATH=%s\n", input);
    } else {
        fprintf(config, "DATAPATH=\n");
    }

    printf("Povolit SERVICELOGS? (1=ano, 0=ne): ");
    scanf("%s", input);
    fprintf(config, "SERVICELOGS=%s\n", input);
    if (strcmp(input, "1") == 0) {
        printf("Zadej cestu k SERVICELOG souboru: ");
        scanf("%s", input);
        fprintf(config, "SERVICEPATH=%s\n\n", input);
    } else {
        fprintf(config, "SERVICEPATH=\n\n");
    }

    // --- Periodické zprávy ---
    printf("Zadej periodu posílaných zpráv (v sekundách): ");
    scanf("%s", input);
    fprintf(config, "PERIOD=%s\n", input);

    if (strcmp(role, "SERVER") == 0) {
        printf("Zadej hodnoty nastavení spontánních zpráv (např. 1;4;7): ");
        scanf("%s", input);
        fprintf(config, "SPONTANEOUS=%s\n", input);

        printf("Zadej hodnotu multiplikátoru: ");
        scanf("%s", input);
        fprintf(config, "MULTI=%s\n\n", input);
    }

    // --- Klient-specifické ---
    if (strcmp(role, "CLIENT") == 0) {
        printf("Zapnout synchronizační zprávy? (1=ano, 0=ne): ");
        scanf("%s", input);
        fprintf(config, "SYNC=%s\n", input);

        printf("Odpojit se po přečtení? (1=ano, 0=ne): ");
        scanf("%s", input);
        fprintf(config, "DISCONNECTAFTERSEND=%s\n\n", input);
    }

    getchar(); // odstraní zbylé '\n' po posledním scanf()

    if (strcmp(role, "SERVER") == 0) {
        allowMessages = true;
    }
    else if (strcmp(role, "CLIENT") == 0) {
        // Povolit jen typy 45 nebo 46
        printf("Chceš odesílat příkazy typu 45/46 jako klient? (1=ano, 0=ne): ");
        fgets(input, sizeof(input), stdin);
        if (input[0] == '1') {
            allowMessages = true;
        }
    }

    if (allowMessages) {
        fprintf(config, "MESSAGES=\n");
        while (1) {
            printf("Zadej zprávu ve formátu TYPE;IOA;VALUE nebo 'k' pro konec: ");
            fgets(input, sizeof(input), stdin);
            input[strcspn(input, "\n")] = 0;

            if (strcmp(input, "k") == 0 || strcmp(input, "K") == 0)
                break;

            int type, ioa;
            float value;

            if (sscanf(input, "%d;%d;%f", &type, &ioa, &value) == 3) {
                if (strcmp(role, "CLIENT") == 0 && (type != 45 && type != 46)) {
                    printf("Klient může odesílat jen zprávy typu 45 nebo 46!\n");
                } else {
                    fprintf(config, "%d;%d;%.2f\n", type, ioa, value);
                }
            } else {
                printf("Neplatný formát, zadej znovu (např. 45;100;1)\n");
            }
        }
    }
    fclose(config);

    if (configInterrupted) {
        remove(tempPath);  // Smazat nedokončený soubor
        return;
    }

    // Bez přerušení → přejmenujeme na finální konfigurák
    if (rename(tempPath, configPath) != 0) {
        perror("Nepodařilo se uložit konfiguraci");
    } else {
        printf("Konfigurace uložena do %s\n", configPath);
    }
}


// Vytiskne obsah konfiguračního souboru na konzoli
void showConfiguration(const char* configPath) {
    FILE* file = fopen(configPath, "r");
    if (!file) {
        printf("Konfigurační soubor '%s' nebyl nalezen.\n", configPath);
        return;
    }
    printf("\n=== OBSAH KONFIGURAČNÍHO SOUBORU (%s) ===\n", configPath);
    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || strlen(line) == 1) continue; // Přeskoč komentáře/prázdné řádky
        printf("%s", line);
    }
    printf("=== KONEC ===\n");
    fclose(file);
}



void printHelp() {
    printf("Použití: ./iec_simulator [možnosti]\n\n");
    printf("Možnosti:\n");
    printf("  --helpconfig      Zobrazí nápovědu k editování konfiguračnáho souboru\n");
    printf("  --edit      Spustí interaktivního průvodce pro vytvoření/úpravu iec_config.txt\n");
    printf("  --show      Zobrazí obsah aktuálního iec_config.txt\n");
    printf("  --help      Zobrazí tuto nápovědu\n\n");
    printf("Bez parametrů se program pokusí načíst konfiguraci a spustit příslušný simulátor.\n");
}

void printHelpConfig() {
    printf("=== Nápověda k parametrům konfigurace (iec_config.txt) ===\n\n");

    printf("PROTOCOL = 101 / 104\n");
    printf("  - Volí mezi protokoly IEC 60870-5-101 (sériový) a 104 (TCP/IP).\n\n");

    printf("ROLE = SERVER / CLIENT\n");
    printf("  - SERVER (SLAVE): odpovídá na dotazy nebo posílá spontánní zprávy.\n");
    printf("  - CLIENT (MASTER): posílá dotazy, vyžaduje data (nebo příkazy typu 45/46).\n\n");

    printf("IP / PORT\n");
    printf("  - Používá se pro IEC 104.\n");
    printf("  - IP = IPv4 adresa serveru.\n");
    printf("  - PORT = TCP port (standardně 2404).\n\n");

    printf("INTERFACE / BANDWIDTH\n");
    printf("  - Používá se pro IEC 101.\n");
    printf("  - INTERFACE = sériové rozhraní, které bude použito pro komunikaci (např. /dev/ttyS0),\n");
    printf("    BANDWIDTH = přenosová rychlost v bps (např. 9600).\n\n");

    printf("ORIGINATOR_ADDRESS = 0–255\n");
    printf("  - Identifikátor zdroje zpráv, používá se v hlavičce ASDU.\n");
    printf("  - Povinný pro SERVER, ale lze použít i na straně CLIENT pro identifikaci zdroje zpráv.\n\n");

    printf("COMMON_ADDRESS = 0–65535\n");
    printf("  - Adresa stanice nebo logické skupiny.\n");
    printf("  - Musí být stejná na obou stranách.\n\n");

    printf("DATALOGS / SERVICELOGS = 0/1\n");
    printf("  - Zapnutí/vypnutí ukládání datových nebo servisních logů.\n\n");

    printf("DATAPATH / SERVICEPATH = cesta k log souborům\n\n");

    printf("PERIOD = celé číslo\n");
    printf("  - Interval mezi periodickým odesíláním zpráv (např. 6 = každých 6s).\n\n");

    printf("SPONTANEOUS = x;y;z\n");
    printf("  - x = 0/1 - vypnuto/zapnuto.\n");
    printf("  - y = celé číslo - min. interval v sekundách pro poslání spontánní zprávy.\n");
    printf("  - z = celé číslo - max. interval v sekundách pro poslání spontánní zprávy.\n");
    printf("  - Aktivní pouze pro SERVER.\n\n");

    printf("MULTI = celé číslo\n");
    printf("  - Počet opakování každé zprávy (např. 2 = každá zpráva bude poslána 2×).\n\n");

    printf("SYNC = 0/1\n");
    printf("  - Povolit (1) nebo zakázat (0) posílání synchronizačních zpráv klientem.\n\n");

    printf("DISCONNECTAFTERSEND = 0/1\n");
    printf("  - Pokud je 1, klient ukončí spojení po přijetí dat od serveru, pokud 0, klient zůstane aktivní do ukončení spojení.\n\n");

    printf("Typy zpráv a hodnoty (MESSAGES):\n");
    printf("  Formát: TYPE;IOA;VALUE\n\n");

    printf("  +------+--------------------------------------------------------------+-------------------------------+\n");
    printf("  | Typ  | Popis                                                       | Povolené hodnoty             |\n");
    printf("  +------+--------------------------------------------------------------+-------------------------------+\n");
    printf("  | 1    | Single Point Information                                     | 0/1 (false/true)             |\n");
    printf("  | 2    | Single Point w/ CP24Time2a                                   | 0/1 (false/true)             |\n");
    printf("  | 3    | Double Point Information                                     | 0/1/2/3 (INTERM/OFF/ON/IND)  |\n");
    printf("  | 4    | Double Point w/ CP24Time2a                                   | 0/1/2/3 (INTERM/OFF/ON/IND)  |\n");
    printf("  | 5    | Step Position (kroková poloha)                               | -64 až +63; transient 0/1     |\n");
    printf("  | 6    | Step Position w/ CP24Time2a                                  | -64 až +63; transient 0/1     |\n");
    printf("  | 7    | Bitstring32                                                  | 0..0xFFFFFFFF                 |\n");
    printf("  | 8    | Bitstring32 w/ CP24Time2a                                    | 0..0xFFFFFFFF                 |\n");
    printf("  | 9    | Measured Value, Normalized                                   | -1.0 až 1.0                  |\n");
    printf("  | 10   | Measured Value, Norm. w/ CP24Time2a                          | -1.0 až 1.0                  |\n");
    printf("  | 11   | Measured Value, Scaled                                       | -32768 až 32767 (int)        |\n");
    printf("  | 12   | Measured Value, Scaled w/ CP24Time2a                         | -32768 až 32767 (int)        |\n");
    printf("  | 13   | Measured Value, Short float                                  | jakékoliv číslo              |\n");
    printf("  | 14   | Measured Value, Short w/ CP24Time2a                          | jakékoliv číslo              |\n");
    printf("  | 15   | Integrated totals (BCR)                                      | int; volit. carry/adjusted/seq|\n");
    printf("  | 16   | Integrated totals (BCR) w/ CP24Time2a                        | int; volit. carry/adjusted/seq|\n");
    printf("  | 30   | Single Point w/ CP56Time2a                                   | 0/1 (false/true)             |\n");
    printf("  | 31   | Double Point w/ CP56Time2a                                   | 0/1/2/3 (INTERM/OFF/ON/IND)  |\n");
    printf("  | 32   | Step Position w/ CP56Time2a                                  | -64 až +63; transient 0/1     |\n");
    printf("  | 33   | Bitstring32 w/ CP56Time2a                                    | 0..0xFFFFFFFF                 |\n");
    printf("  | 34   | Measured Value, Normalized w/ CP56Time2a                     | -1.0 až 1.0                  |\n");
    printf("  | 35   | Measured Value, Scaled w/ CP56Time2a                         | -32768 až 32767 (int)        |\n");
    printf("  | 36   | Measured Value, Short w/ CP56Time2a                          | jakékoliv číslo              |\n");
    printf("  | 37   | Integrated totals (BCR) w/ CP56Time2a                        | int; volit. carry/adjusted/seq|\n");
    printf("  | 45   | Single Command (pouze klient)                                | 0=OFF, 1=ON                  |\n");
    printf("  | 46   | Double Command (pouze klient)                                | 0/1/2/3 (NP/OFF/ON/NP)       |\n");
    printf("  | 47   | Regulating Step Command                                      | -1/0/+1 (DECR/STOP/INCR)      |\n");
    printf("  | 48   | Setpoint command (normalized)                                | -1.0 až 1.0                   |\n");
    printf("  | 49   | Setpoint command (scaled)                                    | -32768 až 32767 (int)         |\n");
    printf("  | 50   | Setpoint command (short float)                               | libovolné číslo               |\n");
    printf("  | 51   | Bitstring32 command                                          | maska 0..0xFFFFFFFF           |\n");
    printf("  +------+--------------------------------------------------------------+-------------------------------+\n\n");


    printf("\n=== Konec nápovědy ===\n");
}

// Spuštění serveru IEC 104 podle načtené konfigurace
void runServer104(Config cfg) {
    printf("[SERVER - 104] Spuštěn s IP %s, port %d, OA %d, CA %d\n", cfg.ip, cfg.port, cfg.originatorAddress, cfg.commonAddress);
    printf("[SERVER - 104] Perioda: %ds | Multiplier: %d | Spontánní zprávy: %s\n", cfg.period, cfg.multiplier, cfg.spontaneousEnable ? "ANO" : "NE");

    // Zapnutí logování dle configu
    if (cfg.serviceLogs) LogSTART("Server");
    if (cfg.dataLogs) dataConfig = 1;
    if (cfg.serviceLogs) serviceConfig = 1;
    if (strlen(cfg.dataPath) > 0) dataPath = cfg.dataPath;
    if (strlen(cfg.servicePath) > 0) servicePath = cfg.servicePath;

    // Načti parametry pro ASDU
    originatorAddress = cfg.originatorAddress;
    commonAddress = cfg.commonAddress;
    multiplier = cfg.multiplier > 0 ? cfg.multiplier : 1;

    // Spontánní zprávy – povol, nastav min/max interval a naplánuj první
    if (cfg.spontaneousEnable)
        spontaneousEnabled = true;
    minSpontaneousInterval = cfg.spontaneousMin;
    maxSpontaneousInterval = cfg.spontaneousMax;
    scheduleNextSpontaneousMessage();

    int periodicInterval = cfg.period > 0 ? cfg.period : 20;
    readMessageConfig("iec_config.txt");

    // Vytvoření a konfigurace slave serveru
    CS104_Slave slave = CS104_Slave_create(10, 10);
    CS104_Slave_setLocalAddress(slave, cfg.ip);
    CS104_Slave_setLocalPort(slave, cfg.port);
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);
    CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);

    // Nastav handlery pro události a příjem/odeslání zpráv
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    // Spusť server
    CS104_Slave_start(slave);
    lastSentTime = time(NULL);

    // Hlavní smyčka: periodicky posílej zprávy + spontánní pokud mají přijít
    while (running) {
        time_t currentTime = time(NULL);

        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            printf("[SERVER - 104] Posílám periodické zprávy:\n");
            for (int i = 0; i < numMessageConfigs; ++i) {
                MessageConfig msg = messageConfigs[i];
                CS101_ASDU asdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, originatorAddress, commonAddress, false, false);
                // Přidej IO objekty podle konfigurace
                for (int j = 0; j < msg.ioContentCount; ++j) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);
                    CS101_ASDU_addInformationObject(asdu, io);
                    InformationObject_destroy(io);
                }
                // Multiplikace (pošle stejnou zprávu vícekrát)
                for (int k = 0; k < multiplier; ++k) {
                    CS104_Slave_enqueueASDU(slave, asdu);
                    asduTransmitHandler(asdu);
                }

                CS101_ASDU_destroy(asdu);
            }
            lastSentTime = currentTime;
        }
        // Spontánní zprávy
        if (spontaneousEnabled && currentTime >= nextSpontaneousTime) {
            printf("[SERVER - 104] Posílám spontánní zprávu...\n");
            sendSpontaneousMessage104(slave, alParams, multiplier);
            scheduleNextSpontaneousMessage();
        }

        Thread_sleep(1000);
    }
    // Při ukončení
    CS104_Connection_sendStopDT(slave);
    CS104_Slave_destroy(slave);
}

static bool asduReceivedHandler(void *parameter, int address, CS101_ASDU asdu) {
    int type = CS101_ASDU_getTypeID(asdu);
    if (type == 100 || type == 103) {
        // Interrogation nebo sync command – klient je pouze posílá, nikdy nezpracovává jako přijaté!
        return true;
    }


    int cot = CS101_ASDU_getCOT(asdu);
    printf("RECVD ASDU | OA: %d | CA: %d | TYPE: %s(%d) | COT: %d (%s) | IOs: %d\n",
           CS101_ASDU_getOA(asdu),
           CS101_ASDU_getCA(asdu),
           TypeID_toString(CS101_ASDU_getTypeID(asdu)),
           CS101_ASDU_getTypeID(asdu),
           cot,
           getCOTName(cot),
           CS101_ASDU_getNumberOfElements(asdu));

    if (dataConfig == 1) { // Filtration of general messages
        LogRX(CS101_ASDU_getTypeID(asdu), CS101_ASDU_getNumberOfElements(asdu), CS101_ASDU_getOA(asdu),
              CS101_ASDU_getCA(asdu));
    }

    switch (CS101_ASDU_getTypeID(asdu)) {
        case M_SP_NA_1: {
            printf("  single point information:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointInformation io = (SinglePointInformation) CS101_ASDU_getElement(asdu, i);
                if (io == NULL) {
                    printf("Error: Failed to retrieve information object at index %d.\n", i);
                    continue;
                }

                int value = SinglePointInformation_getValue(io);
                bool booleanValue = (value != 0); // Assuming 0 is False, any non-zero is True

                // Display the Information Object Address and the value in a descriptive format
                printf("    IOA: %i value: %s\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       booleanValue ? "True" : "False");

                // Conditionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             value); // Log the original integer value
                }

                SinglePointInformation_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_SP_TA_1: {
            printf("  single point information with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointWithCP24Time2a io = (SinglePointWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                bool value = SinglePointInformation_getValue(io); // Gets the boolean state

                // Print the IOA, value as True/False, and the formatted timestamp
                printf("    IOA: %i value: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value ? "True" : "False");

                // Function to print the timestamp in a readable format
                printCP24Time2a(SinglePointWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwT24(InformationObject_getObjectAddress((InformationObject) io),
                              value, // Logging the actual boolean as 1 (True) or 0 (False)
                              SinglePointWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup after use
                SinglePointWithCP24Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_DP_NA_1: {
            printf("  double point information:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointInformation io = (DoublePointInformation) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue(io);

                const char* valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA and the descriptive value
                printf("    IOA: %i value: %s\n", InformationObject_getObjectAddress((InformationObject) io), valueDescription);

                // Conditionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value; // If your LogRXwoT expects a float, convert int to float here
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), logValue);
                }

                DoublePointInformation_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_DP_TA_1: {
            printf("  double point information with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointWithCP24Time2a io = (DoublePointWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue((DoublePointInformation) io);

                const char* valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA, value description and timestamp
                printf("    IOA: %i value: %s time: ", InformationObject_getObjectAddress((InformationObject) io), valueDescription);

                // Function to print time in a readable format
                printCP24Time2a(DoublePointWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value;
                    LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), value, DoublePointWithCP24Time2a_getTimestamp(io));
                }

                DoublePointWithCP24Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_NA_1: {
            printf("  measured value, normalized value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueNormalized io = (MeasuredValueNormalized)CS101_ASDU_getElement(asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);

                // Display detailed information
                printf("    IOA: %i, Normalized Value: %6.3f\n",
                       InformationObject_getObjectAddress((InformationObject) io), normalizedValue);

                // Optionally log the data if enabled
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             normalizedValue);
                }

                // Cleanup to avoid memory leaks
                MeasuredValueNormalized_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TA_1: {
            printf("  measured normalized value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueNormalizedWithCP24Time2a io =
                        (MeasuredValueNormalizedWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);
                CP24Time2a timestamp = MeasuredValueNormalizedWithCP24Time2a_getTimestamp(io);

                // Display detailed information including time stamp
                printf("    IOA: %i, Normalized Value: %6.3f, Timestamp: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       normalizedValue);
                printCP24Time2a(timestamp);

                // Optionally log the data if enabled
                if (dataConfig == 1) {
                    LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), normalizedValue, MeasuredValueNormalizedWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup to avoid memory leaks
                MeasuredValueNormalizedWithCP24Time2a_destroy(io);
                printf("\n"); // Ensure each entry is on a new line
            }
            break;
        }
        case M_ME_NB_1: {
            printf("  measured value, scaled value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaled io =
                        (MeasuredValueScaled)CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueScaled_getValue(io); // Extract value

                // Print IOA and value with better formatting
                printf("    IOA: %i, Scaled Value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Optionally log the data if data logging is enabled
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), value);
                }

                // Cleanup to avoid memory leaks
                MeasuredValueScaled_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TB_1: {
            printf("  measured value, scaled value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaled io =
                        (MeasuredValueScaled)CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueScaled_getValue(io); // Get the scaled value

                // Print IOA, value and timestamp in a structured way
                printf("    IOA: %i, value: %d, time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);
                printCP24Time2a(MeasuredValueScaledWithCP24Time2a_getTimestamp(io)); // Output the timestamp

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwT24(InformationObject_getObjectAddress((InformationObject) io),
                              value,
                              MeasuredValueScaledWithCP24Time2a_getTimestamp(io)); // Log with timestamp
                }

                // Cleanup after use
                MeasuredValueScaledWithCP24Time2a_destroy(io);
                printf("\n"); // Ensure new line for next data point
            }
            break;
        }
        case M_ME_NC_1: {
            printf("  measured value, short value:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShort io = (MeasuredValueShort)CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueShort_getValue(io); // Get the short value

                // Print the IOA and value with improved formatting for clarity
                printf("    IOA: %i, value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io),
                             value); // Log the actual value
                }

                // Cleanup after use
                MeasuredValueShort_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }

        case M_ME_TC_1: { //TADY TOE DOBRY MYSLIM
            printf("  measured value, short value with CP24Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShortWithCP24Time2a io = (MeasuredValueShortWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueShort_getValue(io); // Get the short value

                // Print the IOA, value, and format the timestamp
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);

                // Function to print the timestamp in a readable format
                printCP24Time2a(MeasuredValueShortWithCP24Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), value, MeasuredValueShortWithCP24Time2a_getTimestamp(io));
                }

                // Cleanup after use
                MeasuredValueShortWithCP24Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }
        case M_SP_TB_1: {
            printf("  single point information with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SinglePointWithCP56Time2a io = (SinglePointWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                bool value = SinglePointInformation_getValue(io); // Gets the boolean state

                // Print the IOA, value as True/False, and the formatted timestamp
                printf("    IOA: %i value: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value ? "True" : "False");

                // Function to print the timestamp in a readable format
                printCP56Time2a(SinglePointWithCP56Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    LogRXwT(InformationObject_getObjectAddress((InformationObject) io), value, SinglePointWithCP56Time2a_getTimestamp(io));
                }

                // Cleanup after use
                SinglePointWithCP56Time2a_destroy(io);
                printf("\n"); // New line for each point for better readability
            }
            break;
        }

        case M_DP_TB_1: {
            printf("  double point information with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                DoublePointWithCP56Time2a io = (DoublePointWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                int value = DoublePointInformation_getValue((DoublePointInformation) io);

                const char* valueDescription;
                switch (value) {
                    case IEC60870_DOUBLE_POINT_INTERMEDIATE: // 0
                        valueDescription = "INTERMEDIATE";
                        break;
                    case IEC60870_DOUBLE_POINT_OFF: // 1
                        valueDescription = "OFF";
                        break;
                    case IEC60870_DOUBLE_POINT_ON: // 2
                        valueDescription = "ON";
                        break;
                    case IEC60870_DOUBLE_POINT_INDETERMINATE: // 3
                        valueDescription = "INDETERMINATE";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                // Displaying the IOA, descriptive value and timestamp
                printf("    IOA: %i value: %s time: ", InformationObject_getObjectAddress((InformationObject) io), valueDescription);

                // Function to print time in a readable format
                printCP56Time2a(DoublePointWithCP56Time2a_getTimestamp(io));

                // Optionally log the data if logging is enabled
                if (dataConfig == 1) {
                    float logValue = (float) value;
                    LogRXwT(InformationObject_getObjectAddress((InformationObject) io), value,
                            DoublePointWithCP56Time2a_getTimestamp(io));
                }

                DoublePointWithCP56Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is visually separated
            }
            break;
        }
        case M_ME_TD_1: {
            printf("  measured normalized value with CP56Time2a timestamp:\n");
            int i;

            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaledWithCP56Time2a io = (MeasuredValueScaledWithCP56Time2a)CS101_ASDU_getElement(asdu, i);
                float normalizedValue = MeasuredValueNormalized_getValue(io);

                printf("    IOA: %i, Normalized Value: %6.3f ",
                       InformationObject_getObjectAddress((InformationObject) io), normalizedValue);
                printCP56Time2a(MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogRXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueScaled_getValue((MeasuredValueScaled) io),
                            MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                }

                MeasuredValueScaledWithCP56Time2a_destroy(io);
                printf("\n");  // Ensuring each entry is separated clearly
            }
            break;
        }
        case M_ME_TE_1: {
            printf("  measured scaled value with CP56Time2a timestamp:\n");

            int i;

            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueScaledWithCP56Time2a io = (MeasuredValueScaledWithCP56Time2a)CS101_ASDU_getElement(asdu, i);
                int value = MeasuredValueScaled_getValue(io); // Extract value
                // Assuming the values are scaled and should be displayed as floating-point for better precision
                printf("    IOA: %i, value: %d, time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       value);
                printCP56Time2a(MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogRXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueScaled_getValue((MeasuredValueScaled) io),
                            MeasuredValueScaledWithCP56Time2a_getTimestamp(io));
                }
                MeasuredValueScaledWithCP56Time2a_destroy(io);
                printf("\n"); // Adding a newline for better readability between entries
            }
            break;
        }
        case M_ME_TF_1: {
            printf("  measured short float value with CP56Time2a timestamp:\n");
            int i;
            for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                MeasuredValueShortWithCP56Time2a io = (MeasuredValueShortWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                printf("    IOA: %i value: %.2f time: ",
                       InformationObject_getObjectAddress((InformationObject) io),
                       MeasuredValueShort_getValue((MeasuredValueShort) io));
                printCP56Time2a(MeasuredValueShortWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) {
                    LogRXwT(InformationObject_getObjectAddress((InformationObject) io),
                            MeasuredValueShort_getValue((MeasuredValueShort) io),
                            MeasuredValueShortWithCP56Time2a_getTimestamp(io));
                }
                MeasuredValueShortWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
            /* === NEW RX: Step position (5,6,32) === */
        case M_ST_NA_1: {
            printf("  step position information:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionInformation io = (StepPositionInformation) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue(io);
                bool tr = StepPositionInformation_isTransient(io);
                printf("    IOA: %i value: %d transient: %s\n",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                if (dataConfig == 1) LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), v);
                StepPositionInformation_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_ST_TA_1: {
            printf("  step position information with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionWithCP24Time2a io = (StepPositionWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue((StepPositionInformation) io);
                bool tr = StepPositionInformation_isTransient((StepPositionInformation) io);
                printf("    IOA: %i value: %d transient: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                printCP24Time2a(StepPositionWithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), v,
                                               StepPositionWithCP24Time2a_getTimestamp(io));
                StepPositionWithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_ST_TB_1: {
            printf("  step position information with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                StepPositionWithCP56Time2a io = (StepPositionWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                int v = StepPositionInformation_getValue((StepPositionInformation) io);
                bool tr = StepPositionInformation_isTransient((StepPositionInformation) io);
                printf("    IOA: %i value: %d transient: %s time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v, tr ? "true" : "false");
                printCP56Time2a(StepPositionWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT(InformationObject_getObjectAddress((InformationObject) io), v,
                                             StepPositionWithCP56Time2a_getTimestamp(io));
                StepPositionWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

            /* === NEW RX: Bitstring32 (7,8,33) === */
        case M_BO_NA_1: {
            printf("  bitstring32 value:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                BitString32 io = (BitString32) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue(io);
                printf("    IOA: %i value: 0x%08x\n",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                if (dataConfig == 1) LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)v);
                BitString32_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_BO_TA_1: {
            printf("  bitstring32 value with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                Bitstring32WithCP24Time2a io = (Bitstring32WithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue((BitString32) io);
                printf("    IOA: %i value: 0x%08x time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                printCP24Time2a(Bitstring32WithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), (float)v,
                                               Bitstring32WithCP24Time2a_getTimestamp(io));
                Bitstring32WithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }
        case M_BO_TB_1: {
            printf("  bitstring32 value with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                Bitstring32WithCP56Time2a io = (Bitstring32WithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                uint32_t v = BitString32_getValue((BitString32) io);
                printf("    IOA: %i value: 0x%08x time: ",
                       InformationObject_getObjectAddress((InformationObject) io), v);
                printCP56Time2a(Bitstring32WithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT(InformationObject_getObjectAddress((InformationObject) io), (float)v,
                                             Bitstring32WithCP56Time2a_getTimestamp(io));
                Bitstring32WithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

            /* === NEW RX: Integrated totals / BCR (15,16,37) === */
        case M_IT_NA_1: {
            printf("  integrated totals (BCR):\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotals io = (IntegratedTotals) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR(io);
                int32_t val = BinaryCounterReading_getValue(b);
                printf("    IOA: %i value: %d\n",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                if (dataConfig == 1) LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), (float)val);
                IntegratedTotals_destroy(io);
                printf("\n");
            }
            break;
        }

        case M_IT_TA_1: {
            printf("  integrated totals (BCR) with CP24Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotalsWithCP24Time2a io = (IntegratedTotalsWithCP24Time2a) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR((IntegratedTotals) io);
                int32_t val = BinaryCounterReading_getValue(b);
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                printCP24Time2a(IntegratedTotalsWithCP24Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT24(InformationObject_getObjectAddress((InformationObject) io), (float)val,
                                               IntegratedTotalsWithCP24Time2a_getTimestamp(io));
                IntegratedTotalsWithCP24Time2a_destroy(io);
                printf("\n");
            }
            break;
        }

        case M_IT_TB_1: {
            printf("  integrated totals (BCR) with CP56Time2a:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                IntegratedTotalsWithCP56Time2a io = (IntegratedTotalsWithCP56Time2a) CS101_ASDU_getElement(asdu, i);
                BinaryCounterReading b = IntegratedTotals_getBCR((IntegratedTotals) io);
                int32_t val = BinaryCounterReading_getValue(b);
                printf("    IOA: %i value: %d time: ",
                       InformationObject_getObjectAddress((InformationObject) io), val);
                printCP56Time2a(IntegratedTotalsWithCP56Time2a_getTimestamp(io));
                if (dataConfig == 1) LogRXwT(InformationObject_getObjectAddress((InformationObject) io), (float)val,
                                             IntegratedTotalsWithCP56Time2a_getTimestamp(io));
                IntegratedTotalsWithCP56Time2a_destroy(io);
                printf("\n");
            }
            break;
        }




    }
    return true;
}


InformationObject createIO_client(int messageType, int ioa, float value) {
    InformationObject io;
    switch (messageType) {
        case 45: {
            bool valbool = (value != 0.0) ? true : false;
            io = (InformationObject) SingleCommand_create(NULL, ioa, valbool, 1, 0);
            break;
        }
        case 46: {
            int valint = (int) value;
            io = (InformationObject) DoubleCommand_create(NULL, ioa, valint, 1, 0);
            break;
        }
        case 47: { /* C_RC_NA_1 Regulating step command */
            /* mapuj vstupní value: <0 => LOWER, >0 => HIGHER, ==0 => INVALID_0 */
            StepCommandValue scv = IEC60870_STEP_INVALID_0;
            if (value > 0.5f)        scv = IEC60870_STEP_HIGHER;
            else if (value < -0.5f)  scv = IEC60870_STEP_LOWER;

            /* select=false (EXECUTE), QOC=0 pokud zatím neřešíš kvalifikátor */
            io = (InformationObject) StepCommand_create(
                    NULL, ioa, scv, /*select*/false, /*QOC*/0);
            break;
        }
        case 48: { // C_SE_NA_1 setpoint normalized (-1..1)
            io = (InformationObject) SetpointCommandNormalized_create(NULL, ioa, value, /*select*/false, /*QL*/0);
            break;
        }
        case 49: { // C_SE_NB_1 setpoint scaled (int)
            io = (InformationObject) SetpointCommandScaled_create(NULL, ioa, (int)value, /*select*/false, /*QL*/0);
            break;
        }
        case 50: { // C_SE_NC_1 setpoint short float
            io = (InformationObject) SetpointCommandShort_create(NULL, ioa, value, /*select*/false, /*QL*/0);
            break;
        }
        case 51: { // C_BO_NA_1 bitstring command (32 bit mask)
            io = (InformationObject) Bitstring32Command_create(NULL, ioa, (uint32_t)((int)value));
            break;
        }
    }
    return io;
}

// Spuštění klienta IEC 104 podle konfigurace
void runClient104(Config cfg) {
    printf("[CLIENT - 104] Připojuji se na server %s:%d, CA %d, OA %d\n", cfg.ip, cfg.port, cfg.commonAddress,
           cfg.originatorAddress);
    printf("[CLIENT - 104] Perioda: %ds | SYNC: %s | DisconnectAfterSend: %s | DataLog: %s | ServiceLog: %s\n",
           cfg.period,
           cfg.sync ? "ANO" : "NE",
           cfg.disconnectAfterSend ? "ANO" : "NE",
           cfg.dataLogs ? "ANO" : "NE",
           cfg.serviceLogs ? "ANO" : "NE"
    );

    if (cfg.dataLogs) dataConfig = 1;
    if (cfg.serviceLogs) serviceConfig = 1;
    if (strlen(cfg.dataPath) > 0) dataPath = cfg.dataPath;
    if (strlen(cfg.servicePath) > 0) servicePath = cfg.servicePath;

    if (serviceConfig == 1) {
        LogSTART("Client");
    }


    originatorAddress = cfg.originatorAddress;
    int periodicInterval = cfg.period > 0 ? cfg.period : 20;
    int syncSwitch = cfg.sync;
    int discaftersendSwitch = cfg.disconnectAfterSend;

    typedef struct {
        int messageType;
        int ioa;
        bool toggleState;
    } ToggleBackup;

    ToggleBackup toggleBackup[MAX_MESSAGE_CONFIGS];
    int numToggleBackup = 0;

    lastSentTime = time(NULL);

    // Připrav spojení, ale ještě se nemusí připojit (NULL znamená nepřipojený stav)
    con = NULL;
    running = true;

    // Vytvoření spojení
    con = CS104_Connection_create(cfg.ip, cfg.port);
    CS101_AppLayerParameters alParams = CS104_Connection_getAppLayerParameters(con);
    alParams->originatorAddress = cfg.originatorAddress;

    // Nastavení callbacků
    CS104_Connection_setConnectionHandler(con, connectionHandler, NULL);
    CS104_Connection_setASDUReceivedHandler(con, asduReceivedHandler, NULL);


    if (!CS104_Connection_connect(con)) {
        printf("Connect failed!\n");
        CS104_Connection_destroy(con);
        con = NULL;
    } else {
        CS104_Connection_sendStartDT(con);
        Thread_sleep(1000);

        // 1. Nejprve načti config
        readMessageConfig("iec_config.txt");

        // 2. Hlavní smyčka klienta
        while (running) {
            time_t currentTime = time(NULL);

            if (difftime(currentTime, lastSentTime) >= periodicInterval) {

                // --- Pokud je spojení zavřené, zkus znovu připojit ---
                if (con == NULL) {
                    con = CS104_Connection_create(cfg.ip, cfg.port);
                    CS104_Connection_setConnectionHandler(con, connectionHandler, NULL);
                    CS104_Connection_setASDUReceivedHandler(con, asduReceivedHandler, NULL);
                    if (!CS104_Connection_connect(con)) {
                        printf("Connect failed!\n");
                        CS104_Connection_destroy(con);
                        con = NULL;
                        Thread_sleep(1000);
                        lastSentTime = currentTime;
                        continue;
                    } else {
                        CS104_Connection_sendStartDT(con);
                        Thread_sleep(1000);
                    }
                }

                // === Odeslání commandů 45/46 ===
                bool toDelete[MAX_MESSAGE_CONFIGS] = {false};

                numToggleBackup = 0;
                for (int i = 0; i < numMessageConfigs; ++i) {
                    MessageConfig *msg = &messageConfigs[i];
                    for (int j = 0; j < msg->ioContentCount; ++j) {
                        IOContent *ioContent = &msg->ioContent[j];
                        float valueToSend = ioContent->toggleEnabled
                                            ? (ioContent->toggleState ? ioContent->toggleValueB : ioContent->toggleValueA)
                                            : ioContent->value;


                        InformationObject io = createIO_client(msg->messageType, ioContent->ioa, valueToSend);
                        CS104_Connection_sendProcessCommandEx(con, CS101_COT_ACTIVATION, cfg.commonAddress, io);

                        printf("[CLIENT - 104] Sent command: TYPE=%d | IOA=%d | VALUE=%.2f (%s)\n",
                               msg->messageType,
                               ioContent->ioa,
                               valueToSend,
                               msg->isPermanent ? (ioContent->toggleEnabled ? "PERM-DUAL" : "PERM") : "TEMP"
                        );
                        if (dataConfig == 1) {
                            LogTX(msg->messageType, 1, cfg.originatorAddress, cfg.commonAddress);
                            LogTXwoT(ioContent->ioa, valueToSend);
                        }
                        InformationObject_destroy(io);


                        if (msg->isPermanent && ioContent->toggleEnabled) {
                            ioContent->toggleState = !ioContent->toggleState;
                        }


                        if (ioContent->toggleEnabled) {
                            toggleBackup[numToggleBackup].messageType = msg->messageType;
                            toggleBackup[numToggleBackup].ioa = ioContent->ioa;
                            toggleBackup[numToggleBackup].toggleState = ioContent->toggleState;
                            numToggleBackup++;
                        }


                        if (!msg->isPermanent)
                            toDelete[i] = true;
                    }
                }


                // === Smazání TEMP_MESS z konfigu (pouze odeslané TEMP) ===
                FILE *file = fopen("iec_config.txt", "r");
                FILE *temp = fopen("iec_config_temp.txt", "w");
                if (file && temp) {
                    char line[256];
                    bool insideTempBlock = false;

                    while (fgets(line, sizeof(line), file)) {
                        if (strncmp(line, "TEMP_MESS=", 10) == 0) {
                            insideTempBlock = true;
                            fputs(line, temp); // zachovej TEMP_MESS=
                            continue;
                        }
                        if (strncmp(line, "PERM_MESS=", 10) == 0) {
                            insideTempBlock = false;
                            fputs(line, temp); // zachovej PERM_MESS=
                            continue;
                        }

                        if (insideTempBlock) {
                            int msgType, ioa;
                            float value;
                            // Porovnávej s TEMP zprávami označenými ke smazání
                            if (sscanf(line, "%d;%d;%f", &msgType, &ioa, &value) == 3) {
                                bool found = false;
                                for (int k = 0; k < numMessageConfigs; ++k) {
                                    if (!messageConfigs[k].isPermanent &&
                                        toDelete[k] &&
                                        messageConfigs[k].messageType == msgType &&
                                        messageConfigs[k].ioContent[0].ioa == ioa) {
                                        found = true;
                                        break;
                                    }
                                }
                                if (found) continue; // tuto TEMP zprávu již nezapisuj
                            }
                        }

                        // Vše ostatní přepiš
                        fputs(line, temp);
                    }

                    fclose(file);
                    fclose(temp);
                    remove("iec_config.txt");
                    rename("iec_config_temp.txt", "iec_config.txt");
                }


                // === AŽ TEĎ znovu načti config, už tam TEMP nejsou ===
                readMessageConfig("iec_config.txt");

                // --- Obnov toggleState zpět pro DUAL zprávy ---
                for (int i = 0; i < numMessageConfigs; ++i) {
                    MessageConfig *msg = &messageConfigs[i];
                    for (int j = 0; j < msg->ioContentCount; ++j) {
                        IOContent *ioContent = &msg->ioContent[j];
                        if (ioContent->toggleEnabled) {
                            for (int k = 0; k < numToggleBackup; ++k) {
                                if (toggleBackup[k].messageType == msg->messageType &&
                                    toggleBackup[k].ioa == ioContent->ioa) {
                                    ioContent->toggleState = toggleBackup[k].toggleState;
                                    break;
                                }
                            }
                        }
                    }
                }

                // === Odeslat SYNC (pokud zapnuto) ===
                if (syncSwitch == 1) {
                    struct sCP56Time2a newTime;
                    CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());
                    printf("[CLIENT - 104] Sync command sent\n");
                    CS104_Connection_sendClockSyncCommand(con, cfg.commonAddress, &newTime);
                }

                // === Odeslat INTERROGATION ===
                CS104_Connection_sendInterrogationCommand(con, CS101_COT_ACTIVATION, cfg.commonAddress,
                                                          IEC60870_QOI_STATION);
                printf("[CLIENT - 104] Interrogation command sent\n");
                if (serviceConfig == 1) LogTXrequest(IEC60870_QOI_STATION);

                // === Po periodě případně zavři spojení ===
                if (discaftersendSwitch == 1) {
                    Thread_sleep(1000); // nech přijít odpovědi
                    printf("[CLIENT - 104] Disconnecting after read\n");
                    CS104_Connection_close(con);
                    CS104_Connection_destroy(con);
                    con = NULL;
                }

                lastSentTime = currentTime;
            }
            Thread_sleep(500);
        }

        // Při ukončení aplikace spojení ukliď (pokud je ještě otevřené)
        if (con) {
            CS104_Connection_close(con);
            CS104_Connection_destroy(con);
            con = NULL;
        }
    }
}


void runServer101(Config cfg) {
    printf("[SERVER - 101] Spuštěn na rozhraní %s (baudrate %d), OA %d, CA %d\n",
           cfg.interface, cfg.bandwidth, cfg.originatorAddress, cfg.commonAddress);
    printf("[SERVER - 101] Perioda: %ds | Multiplier: %d | Spontánní zprávy: %s\n",
           cfg.period, cfg.multiplier, cfg.spontaneousEnable ? "ANO" : "NE");

    // Nastavení logování a cest
    if (cfg.dataLogs) dataConfig = 1;
    if (cfg.serviceLogs) serviceConfig = 1;
    if (strlen(cfg.dataPath) > 0) dataPath = cfg.dataPath;
    if (strlen(cfg.servicePath) > 0) servicePath = cfg.servicePath;
    if (serviceConfig == 1) LogSTART("Server101");

    // Adresy a multiplikátor
    originatorAddress = cfg.originatorAddress;
    commonAddress = cfg.commonAddress;
    multiplier = (cfg.multiplier > 0) ? cfg.multiplier : 1;

    // Spontánní zprávy
    if (cfg.spontaneousEnable) spontaneousEnabled = true;
    minSpontaneousInterval = cfg.spontaneousMin;
    maxSpontaneousInterval = cfg.spontaneousMax;
    scheduleNextSpontaneousMessage();

    int periodicInterval = cfg.period > 0 ? cfg.period : 20;
    readMessageConfig("iec_config.txt");

    // === Otevření sériového portu ===
    SerialPort port = SerialPort_create(cfg.interface, cfg.bandwidth, 8, 'E', 1);
    if (!SerialPort_open(port)) {
        printf("Chyba: Nepodařilo se otevřít sériový port %s!\n", cfg.interface);
        return;
    }

    // === Inicializace IEC101 slave ===
    CS101_Slave slave = CS101_Slave_create(port, NULL, NULL, IEC60870_LINK_LAYER_BALANCED);

    // Nastavení linkových adres (pro tvůj use case pravděpodobně 1, případně upravit podle configu)
    CS101_Slave_setLinkLayerAddress(slave, 1);           // adresa stanice (slave)
    CS101_Slave_setLinkLayerAddressOtherStation(slave, 2); // adresa mastera (klienta)

    // Handlery (společné s 104 pokud už máš stejné prototypy)
    CS101_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
    CS101_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);
    CS101_Slave_setASDUHandler(slave, asduHandler, NULL);

    // 101 specifické handlery (nutné!):
    CS101_Slave_setLinkLayerStateChanged(slave, linkLayerStateChanged, NULL);
    CS101_Slave_setResetCUHandler(slave, resetCUHandler, (void*)slave);

    CS101_AppLayerParameters alParams = CS101_Slave_getAppLayerParameters(slave);

    lastSentTime = time(NULL);

    // === Hlavní cyklus ===
    while (running) {
        time_t currentTime = time(NULL);

        // Zpracuje příchozí zprávy (blokuje dokud něco nepřijde nebo timeout, takže krátký sleep na idle)
        CS101_Slave_run(slave);
        Thread_sleep(100);

        // Periodické zprávy
        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            printf("[SERVER - 101] Posílám periodické zprávy:\n");
            for (int i = 0; i < numMessageConfigs; ++i) {
                MessageConfig msg = messageConfigs[i];
                CS101_ASDU asdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC,
                                                    originatorAddress, commonAddress, false, false);
                for (int j = 0; j < msg.ioContentCount; ++j) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);
                    CS101_ASDU_addInformationObject(asdu, io);
                    InformationObject_destroy(io);
                }
                for (int k = 0; k < multiplier; ++k) {
                    CS101_Slave_enqueueUserDataClass1(slave, asdu);
                    asduTransmitHandler(asdu);
                }
                CS101_ASDU_destroy(asdu);
            }
            lastSentTime = currentTime;
        }

        // Spontánní zprávy
        if (spontaneousEnabled && currentTime >= nextSpontaneousTime) {
            printf("[SERVER - 101] Posílám spontánní zprávu...\n");
            sendSpontaneousMessage101(slave, alParams, multiplier);
            scheduleNextSpontaneousMessage();
        }
    }

    // Ukončení serveru
    CS101_Slave_destroy(slave);
    SerialPort_close(port);
    SerialPort_destroy(port);
}


void runClient101(Config cfg) {
    originatorAddress = cfg.originatorAddress;
    printf("[CLIENT - 101] Připojuji se na rozhraní %s (baudrate %d), CA %d\n", cfg.interface, cfg.bandwidth, cfg.commonAddress);
    printf("[CLIENT - 101] Perioda: %ds | SYNC: %s | DataLog: %s | ServiceLog: %s\n",
           cfg.period,
           cfg.sync ? "ANO" : "NE",
           cfg.dataLogs ? "ANO" : "NE",
           cfg.serviceLogs ? "ANO" : "NE"
    );

    if (cfg.dataLogs) dataConfig = 1;
    if (cfg.serviceLogs) serviceConfig = 1;
    if (strlen(cfg.dataPath) > 0) dataPath = cfg.dataPath;
    if (strlen(cfg.servicePath) > 0) servicePath = cfg.servicePath;

    if (serviceConfig == 1) LogSTART("Client101");

    int periodicInterval = cfg.period > 0 ? cfg.period : 20;
    int syncSwitch = cfg.sync;
    bool toDelete[MAX_MESSAGE_CONFIGS] = {false};
    int deleteCount = 0;

    lastSentTime = time(NULL);
    running = true;

    // --- Otevření sériového portu ---
    SerialPort port = SerialPort_create(cfg.interface, cfg.bandwidth, 8, 'E', 1);
    if (!SerialPort_open(port)) {
        printf("Chyba: Nepodařilo se otevřít sériový port %s!\n", cfg.interface);
        return;
    }

    // --- Inicializace 101 mastera ---
    CS101_Master master = CS101_Master_create(port, NULL, NULL, IEC60870_LINK_LAYER_BALANCED);

    CS101_Master_setOwnAddress(master, cfg.originatorAddress);
    CS101_Master_useSlaveAddress(master, 1);       // slave adresa (možno z configu)
    CS101_Master_setASDUReceivedHandler(master, asduReceivedHandler, NULL);
    LinkLayerParameters llParams = CS101_Master_getLinkLayerParameters(master);
    llParams->useSingleCharACK = false;
    CS101_Master_setLinkLayerStateChanged(master, linkLayerStateChanged, NULL);

    printf("[CLIENT - 101] Master běží, čekám na periodický interval...\n");

    while (running) {
        CS101_Master_run(master);  // procesuj příchozí zprávy
        time_t currentTime = time(NULL);
        Thread_sleep(100);

        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            readMessageConfig("iec_config.txt");
            for (int i = 0; i < numMessageConfigs; ++i) {
                MessageConfig* msg = &messageConfigs[i];
                if (msg->messageType == 45 || msg->messageType == 46) {
                    for (int j = 0; j < msg->ioContentCount; ++j) {
                        IOContent* ioContent = &msg->ioContent[j];
                        float valueToSend = ioContent->toggleEnabled ?
                                            (ioContent->toggleState ? ioContent->toggleValueB : ioContent->toggleValueA) :
                                            ioContent->value;
                        if (ioContent->toggleEnabled)
                            ioContent->toggleState = !ioContent->toggleState;
                        InformationObject io = createIO_client(msg->messageType, ioContent->ioa, ioContent->value);
                        CS101_Master_sendProcessCommand(master, CS101_COT_ACTIVATION, cfg.commonAddress, io);
                        printf("[CLIENT - 101] Sent command: TYPE=%d IOA=%d VALUE=%.2f (%s)\n",
                               msg->messageType,
                               ioContent->ioa,
                               valueToSend,
                               msg->isPermanent ?
                               (ioContent->toggleEnabled ? "PERM-DUAL" : "PERM")
                                                : "TEMP");
                        if (dataConfig == 1) {
                            LogTX(msg->messageType, 1, cfg.originatorAddress, cfg.commonAddress);
                            LogTXwoT(ioContent->ioa, ioContent->value);
                        }
                        InformationObject_destroy(io);
                    }

                    if (!msg->isPermanent) {
                        toDelete[i] = true;
                        deleteCount++;
                    }
                }
            }

            // === Smazání TEMP_MESS bloků z konfiguračního souboru ===
            FILE *file = fopen("iec_config.txt", "r");
            FILE *temp = fopen("iec_config_temp.txt", "w");
            if (file && temp) {
                char line[256];
                bool insideTempBlock = false;

                while (fgets(line, sizeof(line), file)) {
                    if (strncmp(line, "TEMP_MESS=", 10) == 0) {
                        insideTempBlock = true;
                        fputs(line, temp); // zachovej TEMP_MESS=
                        continue;
                    }
                    if (strncmp(line, "PERM_MESS=", 10) == 0) {
                        insideTempBlock = false;
                        fputs(line, temp); // zachovej PERM_MESS=
                        continue;
                    }

                    if (insideTempBlock) {
                        int msgType;
                        if (sscanf(line, "%d;", &msgType) == 1 && (msgType == 45 || msgType == 46)) {
                            // Smaž TEMP zprávy (nepíšeme do souboru)
                            continue;
                        }
                    }

                    // Vše ostatní přepiš
                    fputs(line, temp);
                }

                fclose(file);
                fclose(temp);
                remove("iec_config.txt");
                rename("iec_config_temp.txt", "iec_config.txt");
            }


            // SYNC
            if (syncSwitch == 1) {
                struct sCP56Time2a newTime;
                CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());
                printf("[CLIENT - 101] Sync command sent\n");
                CS101_Master_sendClockSyncCommand(master, cfg.commonAddress, &newTime);
            }

            // INTERROGATION
            CS101_Master_sendInterrogationCommand(master, CS101_COT_ACTIVATION, cfg.commonAddress, IEC60870_QOI_STATION);
            printf("[CLIENT - 101] Interrogation command sent\n");
            if (serviceConfig == 1) LogTXrequest(IEC60870_QOI_STATION);

            lastSentTime = currentTime;
        }
    }

    CS101_Master_destroy(master);
    SerialPort_close(port);
    SerialPort_destroy(port);
    printf("[CLIENT - 101] Klient ukončen.\n");
}



// Hlavní funkce – spustí editor/show/help nebo konkrétní režim (server/klient)
int main(int argc, char **argv) {
    srand(time(NULL));
    signal(SIGINT, sigint_handler);

    if (argc > 1) {
        if (strcmp(argv[1], "--edit") == 0) {
            runInteractiveEditor("iec_config.txt");
            return 0;
        } else if (strcmp(argv[1], "--show") == 0) {
            showConfiguration("iec_config.txt");
            return 0;
        } else if (strcmp(argv[1], "--help") == 0) {
            printHelp();
            return 0;
        } else if (strcmp(argv[1], "--helpconfig") == 0) {
            printHelpConfig();
            return 0;
        }
    }

    Config cfg = readFullConfig("iec_config.txt");


    if (strcmp(cfg.protocol, "104") == 0 && strcmp(cfg.role, "SERVER") == 0) {
        runServer104(cfg);
    } else if (strcmp(cfg.protocol, "104") == 0 && strcmp(cfg.role, "CLIENT") == 0) {
        runClient104(cfg);
    } else if (strcmp(cfg.protocol, "101") == 0 && strcmp(cfg.role, "SERVER") == 0) {
        runServer101(cfg);
    } else if (strcmp(cfg.protocol, "101") == 0 && strcmp(cfg.role, "CLIENT") == 0) {
        runClient101(cfg);
    } else {
        fprintf(stderr, "Neplatná kombinace PROTOCOL a ROLE v iec_config.txt\n");
        return 1;
    }

    return 0;
}
