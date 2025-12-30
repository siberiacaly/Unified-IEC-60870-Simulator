#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include "hal_serial.h"
#include "cs101_master.h"
#include "cs104_connection.h"
#include "hal_thread.h"
#include "hal_time.h"
#include <time.h>
#include "iec60870_common.h"


#define MAX_MESSAGES 100

int dataConfig = 0;
int serviceConfig = 0;

typedef struct {
    int ioa;
    float value;
} IOContent;

typedef struct {
    int messageType;
    IOContent ioContent[10];
    int ioContentCount; // Keep track of how many IOContent are in this MessageConfig
} MessageConfig;

InformationObject ioBuffer[MAX_MESSAGES];
InformationObject *spontBuffer[MAX_MESSAGES];
CS101_ASDU asduBuffer[MAX_MESSAGES];
int numIoBuffer = 0;
int numAsduBuffer = 0;
int numSpontBuffer = 0;


static MessageConfig messageConfigs[MAX_MESSAGES];
static int numMessageConfigs = 0;


static bool running = true;
static time_t lastSentTime = 0;
bool dataLog = false;


static char *dataPath = "DATALOG.txt";
static char *servicePath = "SERVICELOG.txt";

void
printCP56Time2a(CP56Time2a time) {
    printf("%02i:%02i:%02i %02i/%02i/%04i", CP56Time2a_getHour(time),
           CP56Time2a_getMinute(time),
           CP56Time2a_getSecond(time),
           CP56Time2a_getDayOfMonth(time),
           CP56Time2a_getMonth(time),
           CP56Time2a_getYear(time) + 2000);
}

void printCP24Time2a(CP24Time2a time) {
    printf("%02imin %02isec %02imilisec",
           CP24Time2a_getMinute(time),
           CP24Time2a_getSecond(time),
           CP24Time2a_getMillisecond(time));
}

void
CP24Time2a_setFromMsTimestamp(CP24Time2a self, uint64_t timestamp) {
    memset(self->encodedValue, 0, 7);

    time_t timeVal = timestamp / 1000;
    int msPart = timestamp % 1000;

    struct tm tmTime;

    gmtime_r(&timeVal, &tmTime);

    CP24Time2a_setMillisecond(self, msPart);

    CP24Time2a_setSecond(self, tmTime.tm_sec);

    CP24Time2a_setMinute(self, tmTime.tm_min);
}


void sigint_handler(int signalId) {
    running = false;
}


//general data log
void LogSTART() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Client started \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONREQ(const char *ipAddress) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d New connection request from %s: \n", tm.tm_year + 1900, tm.tm_mon + 1,
            tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ipAddress);
    fclose(fp);
}

void LogCONEST() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection established\n", tm.tm_year + 1900, tm.tm_mon + 1,
            tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONCLOSED() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection closed \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONSTARTTD() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received START_CON \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONSTOPTD() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received STOPDT_CON \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONREF() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connections refreshed \n", tm.tm_year + 1900, tm.tm_mon + 1,
            tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogRX(int type, int elements, int oa, int ca) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d oa: %i ca: %i type:(%i) elements: %i\n", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, oa, ca, type, elements);
    fclose(fp);
}

void LogRXwoT(int ioa, float value) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min,
            tm.tm_sec, ioa, value);
    fclose(fp);
}

void LogRXwT(int ioa, float value, CP56Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %04i-%02i-%02i %02i:%02i:%02i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
            tm.tm_sec, ioa, value, CP56Time2a_getYear(timestamp) + 2000, CP56Time2a_getMonth(timestamp),
            CP56Time2a_getDayOfMonth(timestamp),
            CP56Time2a_getHour(timestamp), CP56Time2a_getMinute(timestamp), CP56Time2a_getSecond(timestamp));
    fclose(fp);
}
void LogRXwT24(int ioa, float value, CP24Time2a timestamp) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f with timestamp: %02i:%02i:%03i \n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
            tm.tm_sec, ioa, value,
            CP24Time2a_getMinute(timestamp), CP24Time2a_getSecond(timestamp), CP24Time2a_getMillisecond(timestamp));
    fclose(fp);
}

//TX data log
void LogTXrequest(int qoi) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Transceived interrogation for group (%i).\n", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, qoi);
    fclose(fp);
}

void LogTX(int type, int elements) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d type(%i) elements: %i\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, type, elements);
    fclose(fp);
}

void LogTXwoT(int ioa, float value) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d IOA: %i value: %f\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min,
            tm.tm_sec, ioa, value);
    fclose(fp);
}


/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void *parameter, uint8_t *msg, int msgSize, bool sent) {
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    int i;
    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

void readMessageConfig(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open configuration file");
        return;
    }

    char line[256];
    int currentMessageType = -1;
    int messageType;
    int ioa;
    float value;
    int ioContentIndex = 0; // Initialize index for IOContent

    // we assume the config lines are sorted in order by message type
    while (fgets(line, sizeof(line), file) != NULL) {
        if (strncmp(line, "MESS=", 5) != 0) {
            // only start reading after MESS tag
            continue;
        }
        while (fgets(line, sizeof(line), file) != NULL) {
            if (sscanf(line, "%d;%d;%f", &messageType, &ioa, &value) == 3) {
                if (ioa > 65535) {
                    printf("%d is not a valid ioa value\n", ioa);
                    continue;
                }
                if (messageType == currentMessageType) {
                    messageConfigs[numMessageConfigs - 1].ioContent[
                            messageConfigs[numMessageConfigs].ioContentCount + 1].ioa = ioa;
                    messageConfigs[numMessageConfigs - 1].ioContent[
                            messageConfigs[numMessageConfigs].ioContentCount + 1].value = value;
                    messageConfigs[numMessageConfigs - 1].ioContentCount += 1;
                } else {
                    currentMessageType = messageType;
                    messageConfigs[numMessageConfigs].messageType = messageType;
                    messageConfigs[numMessageConfigs].ioContent[0].ioa = ioa;
                    messageConfigs[numMessageConfigs].ioContent[0].value = value;
                    messageConfigs[numMessageConfigs].ioContentCount = 1;
                    ++numMessageConfigs;
                }
            }
        }
    }
    fclose(file);
}

void remove_lines_after_mess(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Failed to open file");
        return;
    }

    // Vytvoření dočasného souboru
    char temp_filename[] = "tempfile.txt";
    FILE *temp_file = fopen(temp_filename, "w");
    if (temp_file == NULL) {
        perror("Failed to open temporary file");
        fclose(file);
        return;
    }

    char line[1024];
    int delete_rest = 0;

    while (fgets(line, sizeof(line), file) != NULL) {
        if (strstr(line, "MESS=")) {
            delete_rest = 1;
        }

        if (!delete_rest) {
            fputs(line, temp_file);
        }
    }

    fclose(file);
    fclose(temp_file);

    // Nahradit původní soubor dočasným souborem
    remove(filename);
    rename(temp_filename, filename);
}

char *readConfigValue(const char *filePath, const char *key) {
    FILE *file = fopen(filePath, "r");
    if (file == NULL) {
        perror("Failed to open file");
        return NULL;
    }

    char *value = NULL;
    char line[1024];
    size_t keyLen = strlen(key);

    while (fgets(line, sizeof(line), file)) {
        char *pos = strstr(line, key);
        if (pos != NULL && line[keyLen] == '=') {
            char *startOfValue = pos + keyLen + 1;
            char *endOfValue = startOfValue + strcspn(startOfValue, "\r\n");
            *endOfValue = '\0';
            value = strdup(startOfValue);
            break;
        }
    }

    fclose(file);
    return value;
}


/* Connection event handler */
static void
connectionHandler(void *parameter, CS104_Connection connection, CS104_ConnectionEvent event) {
    switch (event) {
        case CS104_CONNECTION_OPENED: {
            printf("Connection established\n");
            if (serviceConfig == 1) {
                LogCONEST();
            }
            break;
        }
        case CS104_CONNECTION_CLOSED: {
            printf("Connection closed\n");
            if (serviceConfig == 1) {
                LogCONCLOSED();
            }
            break;
        }
        case CS104_CONNECTION_FAILED: {
            printf("Failed to connect\n");
            break;
        }
        case CS104_CONNECTION_STARTDT_CON_RECEIVED: {
            printf("Received STARTDT_CON\n");
            if (serviceConfig == 1) {
                LogCONSTARTTD();
            }
            break;
        }

        case CS104_CONNECTION_STOPDT_CON_RECEIVED: {
            printf("Received STOPDT_CON\n");
            if (serviceConfig == 1) {
                LogCONSTOPTD();
            }
            break;
        }
    }
}

static bool asduReceivedHandler(void *parameter, int address, CS101_ASDU asdu) {
    printf("RECVD ASDU oa: %i ca: %i type: %s(%i) elements: %i \n",
           CS101_ASDU_getOA(asdu),
           CS101_ASDU_getCA(asdu),
           TypeID_toString(CS101_ASDU_getTypeID(asdu)),
           CS101_ASDU_getTypeID(asdu),
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
    }
    return true;
}


InformationObject createIO(int messageType, int ioa, float value) {
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
    }
    return io;
}


int main(int argc, char **argv) {
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

    char *ip = readConfigValue("KONFIG104CLIENT.txt", "IP config");
    char *interface = readConfigValue("KONFIG104CLIENT.txt", "Interface");
    char *portStr = readConfigValue("KONFIG104CLIENT.txt", "Port");
    char *commonAddressStr = readConfigValue("KONFIG104CLIENT.txt", "Common Address");
    char *periodStr = readConfigValue("KONFIG104CLIENT.txt", "PERIOD");
    int periodicInterval = periodStr ? atoi(periodStr)
                                     : 20;  // Defaultní perioda je 20 sekund, pokud není specifikováno jinak
    char *syncStr = readConfigValue("KONFIG104CLIENT.txt", "SYNC");
    int syncSwitch = syncStr ? atoi(syncStr) : 0;
    char *discaftersendStr = readConfigValue("KONFIG104CLIENT.txt", "DISCONNECTAFTERSEND");
    int discaftersendSwitch = discaftersendStr ? atoi(discaftersendStr) : 0;
    char *dataConfigStr = readConfigValue("KONFIG104CLIENT.txt", "DATALOGS");
    dataConfig = atoi(dataConfigStr);
    char *serviceConfigStr = readConfigValue("KONFIG104CLIENT.txt", "SERVICELOGS");
    serviceConfig = atoi(serviceConfigStr);
    servicePath = readConfigValue("KONFIG104CLIENT.txt", "SERVICEPATH");
    dataPath = readConfigValue("KONFIG104CLIENT.txt", "DATAPATH");

    if (serviceConfig == 1) {
        LogSTART();
    }

    free(syncStr);
    free(periodStr);
    free(discaftersendStr);
    FILE *logFile = NULL;


    if (!ip || !interface || !portStr || !commonAddressStr) {
        fprintf(stderr, "Failed to read some configuration values. Exiting...\n");
        free(ip);
        free(interface);
        free(portStr);
        free(commonAddressStr);
        return -1;
    }

    int port = atoi(portStr);
    int commonAddress = atoi(commonAddressStr);

    printf("IP Address: %s\n", ip);
    printf("Interface: %s\n", interface);
    printf("Port: %d\n", port);
    printf("Common Address: %d\n", commonAddress);


    printf("Connecting to: %s:%i\n", ip, port);

    CS104_Connection con = CS104_Connection_create(ip, port);

    CS101_AppLayerParameters alParams = CS104_Connection_getAppLayerParameters(con);
    alParams->originatorAddress = 3;

    CS104_Connection_setConnectionHandler(con, connectionHandler, NULL);
    CS104_Connection_setASDUReceivedHandler(con, asduReceivedHandler, NULL);

    int shouldConnect = discaftersendSwitch;
    lastSentTime = time(NULL);  // Nastavit čas při startu
    /* uncomment to log messages */
    //CS104_Connection_setRawMessageHandler(con, rawMessageHandler, NULL);
    while (running) {
        // spusti se JEDNOU kdyz discaftersend je vyple
        if (shouldConnect == 0) {
            if (!CS104_Connection_connect(con)) {
                printf("Connect failed!\n");
                Thread_sleep(1000);
                CS104_Connection_destroy(con);
                free(ip);
                free(interface);
                free(portStr);
                printf("Exiting...\n");
            }
            CS104_Connection_sendStartDT(con);
            Thread_sleep(2000);
            shouldConnect = 1;
        }
        // odesle zpravy
        time_t currentTime = time(NULL);
        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            readMessageConfig("KONFIG104CLIENT.txt");
            for (int i = 0; i < numMessageConfigs; ++i) {
                CS101_ASDU asdu = CS101_ASDU_create(alParams, true, CS101_COT_REQUEST, 1, commonAddress, false,
                                                    false);
                MessageConfig msg = messageConfigs[i];
                for (int j = 0; j < msg.ioContentCount; ++j) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);
                    if (msg.messageType == 45 || msg.messageType == 46) {
                        CS104_Connection_sendProcessCommandEx(con, CS101_COT_ACTIVATION, commonAddress, io);
                        printf("Sent command number: %d \n", i + 1);
                        if (dataConfig == 1) {
                            LogTX(msg.messageType, 1);
                            LogTXwoT(ioContent.ioa, ioContent.value);
                        }
                    } else {
                        CS101_ASDU_addInformationObject(asdu, io);
                    }
                    InformationObject_destroy(io);
                }
                CS101_ASDU_destroy(asdu);
            }
            memset(messageConfigs, 0, sizeof(messageConfigs));
            const char *config_file = "KONFIG104CLIENT.txt";
            remove_lines_after_mess(config_file);
            // spusti se pokazdy kdyz je disaftersend zaple
            if (discaftersendSwitch == 1) {
                bool connected = CS104_Connection_connect(con);
                if (!connected) {
                    printf("Connect failed!\n");
                    Thread_sleep(1000);
                    CS104_Connection_destroy(con);
                    free(ip);
                    free(interface);
                    free(portStr);
                    printf("Exiting...\n");
                    return -1;
                }
                printf("Connected\n");
                CS104_Connection_sendStartDT(con);
                Thread_sleep(2000);
            }
            if (syncSwitch == 1) {
                struct sCP56Time2a newTime;
                CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());
                printf("Sending time sync command\n");
                CS104_Connection_sendClockSyncCommand(con, commonAddress, &newTime);
            }
            CS104_Connection_sendInterrogationCommand(con, CS101_COT_ACTIVATION, commonAddress, IEC60870_QOI_STATION);
            printf("\n↓↓↓ Interrogation command was sent (type 100) ↓↓↓\n");
            if (serviceConfig == 1) {
                LogTXrequest(IEC60870_QOI_STATION);
            }
            Thread_sleep(3000);
            // kdyz je disaftersend zaple tak se odpoji
            if (discaftersendSwitch == 1) {
                printf("Disconnecting\n");
                CS104_Connection_close(con);
                Thread_sleep(2000);
            }
            lastSentTime = currentTime;  // Update the last sent time
        }
    }
    CS104_Connection_sendStopDT(con);
    printf("Received SIGINT\n");
    Thread_sleep(1000);
    CS104_Connection_destroy(con);
    free(ip);
    free(interface);
    free(portStr);
    printf("Exiting...\n");
    return 0;
}

