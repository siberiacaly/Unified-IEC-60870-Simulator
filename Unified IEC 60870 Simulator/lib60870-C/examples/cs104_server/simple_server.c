#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include "cs104_slave.h"
#include "hal_thread.h"
#include "hal_time.h"
#include <time.h>
#include "iec60870_common.h"
#include "lib_memory.h"
#include <math.h>


#define MAX_MESSAGES 100
#define MAX_MESSAGE_TYPES 40

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

static char *dataPath = "DATALOG.txt";
static char *servicePath = "SERVICELOG.txt";
static MessageConfig messageConfigs[MAX_MESSAGES];
static int numMessageConfigs = 0;
static bool running = true;
static time_t lastSentTime = 0;
static bool spontaneousEnabled = false;
static int minSpontaneousInterval = 2;
static int maxSpontaneousInterval = 10;
static time_t nextSpontaneousTime = 0;
static int multiplier = 1;
static int originatorAddress;
static int commonAddress;
int manualCounts[MAX_MESSAGE_TYPES] = {0};

void sigint_handler(int signalId) {
    running = false;
}


void LogSTART() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Server started \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONREQ(const char *ipAddress) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d New connection request from %s \n", tm.tm_year + 1900, tm.tm_mon + 1,
            tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ipAddress);
    fclose(fp);
}

void LogCONOPEN() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection opened \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
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

void LogCONACT() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection activated \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}

void LogCONDEACT() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Connection deactivated \n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(fp);
}


void LogTX(int type, int elements, int oa, int ca) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d oa: %i ca: %i type:(%i) elements: %i\n", tm.tm_year + 1900, tm.tm_mon + 1,
            tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, oa, ca, type, elements);
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

void LogTXwT(int ioa, float value, CP56Time2a timestamp) {
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

void LogTXwT24(int ioa, float value, CP24Time2a timestamp) {
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


void LogRXrequest(int qoi) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(servicePath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d Received interrogation for group (%i).\n", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, qoi);
    fclose(fp);
}

void LogRX(int type, int elements) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    FILE *fp;
    fp = fopen(dataPath, "a");
    fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d type(%i) elements: %i\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
            tm.tm_hour, tm.tm_min, tm.tm_sec, type, elements);
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

static bool asduTransmitHandler(CS101_ASDU asdu) {
    printf("TRANSMITTED ASDU oa: %i ca: %i type: %s(%i) elements: %i \n",
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
    }
    return true;
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

CP24Time2a CP24Time2a_createFromMsTimestamp(CP24Time2a self, uint64_t msTimestamp) {
    if (self == NULL)
        self = (CP24Time2a) GLOBAL_CALLOC(1, sizeof(struct sCP24Time2a));
    else
        memset(self, 0, sizeof(struct sCP24Time2a));

    if (self != NULL)
        CP24Time2a_setFromMsTimestamp(self, msTimestamp);

    return self;
}

InformationObject createIO(int messageType, int ioa, float value) {
    InformationObject io;
    switch (messageType) {
        case 1: {
            bool valbool = (value != 0.0) ? true : false;
            io = (InformationObject) SinglePointInformation_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD);
            break;
        }
        case 2: {
            bool valbool = (value != 0.0) ? true : false;
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) SinglePointWithCP24Time2a_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD,
                                                                      timestamp);
            break;
        }
        case 3: {
            int valint = (int) value;
            io = (InformationObject) DoublePointInformation_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD);
            break;
        }
        case 4: {
            int valint = (int) value;
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) DoublePointWithCP24Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                      timestamp);
            break;
        }
        case 9: {
            io = (InformationObject) MeasuredValueNormalized_create(NULL, ioa, value, IEC60870_QUALITY_GOOD);
            break;
        }
        case 10: {
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueNormalizedWithCP24Time2a_create(NULL, ioa, value,
                                                                                  IEC60870_QUALITY_GOOD, timestamp);
            break;
        }
        case 11: {
            int valint = (int) value;
            io = (InformationObject) MeasuredValueScaled_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD);
            break;
        }
        case 12: {
            int valint = (int) value;
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueScaledWithCP24Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                              timestamp);
            break;
        }
        case 13: {
            io = (InformationObject) MeasuredValueShort_create(NULL, ioa, value, IEC60870_QUALITY_GOOD);
            break;
        }
        case 14: {
            CP24Time2a timestamp = CP24Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs());
            io = (InformationObject) MeasuredValueShortWithCP24Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD,
                                                                             timestamp);
            break;
        }
        case 30: {
            bool valbool = (value != 0.0) ? true : false;
            io = (InformationObject) SinglePointWithCP56Time2a_create(NULL, ioa, valbool, IEC60870_QUALITY_GOOD,
                                                                      CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                       Hal_getTimeInMs()));
            break;
        }
        case 31: {
            int valint = (int) value;
            io = (InformationObject) DoublePointWithCP56Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                      CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                       Hal_getTimeInMs()));
            break;
        }
        case 34: {
            io = (InformationObject) MeasuredValueNormalizedWithCP56Time2a_create(NULL, ioa, value,
                                                                                  IEC60870_QUALITY_GOOD,
                                                                                  CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                                   Hal_getTimeInMs()));
            break;
        }
        case 35: {
            int valint = (int) value;
            io = (InformationObject) MeasuredValueScaledWithCP56Time2a_create(NULL, ioa, valint, IEC60870_QUALITY_GOOD,
                                                                              CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                               Hal_getTimeInMs()));
            break;
        }
        case 36: {
            io = (InformationObject) MeasuredValueShortWithCP56Time2a_create(NULL, ioa, value, IEC60870_QUALITY_GOOD,
                                                                             CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                              Hal_getTimeInMs()));
            break;
        }
            return io;
    }
}
void readMessageConfig(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open configuration file");
        return;
    }

    char line[256];

    int currentMessageType = -1;

    while (fgets(line, sizeof(line), file) != NULL) {
        int messageType, ioa;
        float value;

        if (strncmp(line, "MESS=", 5) == 0) {
            continue; // Skip the MESS= line
        }

        if (sscanf(line, "%d;%d;%f", &messageType, &ioa, &value) == 3) {
            if (ioa > 65535) {
                printf("%d is not a valid ioa value\n", ioa);
                continue;
            }

            // Check if current message type has changed
            if (messageType != currentMessageType) {
                // If it's not the first entry and message type changed, increase the configuration counter
                if (currentMessageType != -1) {
                    ++numMessageConfigs;
                }
                currentMessageType = messageType;
                messageConfigs[numMessageConfigs].messageType = messageType;
                messageConfigs[numMessageConfigs].ioContentCount = 0;
            }

            // Add new IOContent under the current messageType
            int index = messageConfigs[numMessageConfigs].ioContentCount;
            messageConfigs[numMessageConfigs].ioContent[index].ioa = ioa;
            messageConfigs[numMessageConfigs].ioContent[index].value = value;
            messageConfigs[numMessageConfigs].ioContentCount++;
        }
    }

    // Increment configuration count once more if the last entry was valid
    if (currentMessageType != -1) {
        ++numMessageConfigs;
    }

    fclose(file);
}

void sendSpontaneousMessage(CS104_Slave slave, CS101_AppLayerParameters alparams, int multiplier) {
    CS101_ASDU newAsdu = CS101_ASDU_create(alparams, true, CS101_COT_SPONTANEOUS, originatorAddress, commonAddress,
                                           false, false);
    InformationObject ios[MAX_MESSAGES];
    int numSpontIos = 0;
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
    if (numSpontIos != 0) {
        int index = rand() % (numSpontIos);
        io = ios[index];
    } else {
        io = (InformationObject) SinglePointWithCP56Time2a_create(NULL, 9999, 1, IEC60870_QUALITY_GOOD,
                                                                  CP56Time2a_createFromMsTimestamp(NULL,
                                                                                                   Hal_getTimeInMs()));
    }
    /*if (dataLog == 1){
        LogTXwT(int ioa, float value,CP56Time2a timestamp )
    }*/
    CS101_ASDU_addInformationObject(newAsdu, io);
    for (int i = 0; i < multiplier; ++i) {
        CS104_Slave_enqueueASDU(slave, newAsdu);
        asduTransmitHandler(newAsdu);
    }
    if (numSpontIos == 0) {
        InformationObject_destroy(io);
    }
    CS101_ASDU_destroy(newAsdu);
    printf("Spontaneous messages sent count: %d at %s\n", multiplier, ctime(&nextSpontaneousTime));
}

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

void scheduleNextSpontaneousMessage() {
    if (!spontaneousEnabled) return;
    int interval = minSpontaneousInterval + rand() % (maxSpontaneousInterval - minSpontaneousInterval + 1);
    nextSpontaneousTime = time(NULL) + interval;
}


void
printCP56Time2a(CP56Time2a time) {
    printf("%02i:%02i:%02i %02i/%02i/%04i",
           CP56Time2a_getHour(time),
           CP56Time2a_getMinute(time),
           CP56Time2a_getSecond(time),
           CP56Time2a_getDayOfMonth(time),
           CP56Time2a_getMonth(time),
           CP56Time2a_getYear(time) + 2000);
}


void printCP24Time2a(CP24Time2a time) {
    printf("%02imin %02isec",
           CP24Time2a_getMinute(time),
           CP24Time2a_getSecond(time));
}


/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void *parameter, IMasterConnection connection, uint8_t *msg, int msgSize, bool sent) {
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

static bool
clockSyncHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime) {
    printf("Process time sync command with time ");
    printCP56Time2a(newTime);
    printf("\n");

    uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);


    CP56Time2a_setFromMsTimestamp(newTime, Hal_getTimeInMs());


    return true;
}

static bool interrogationHandler(void *parameter, IMasterConnection connection, CS101_ASDU requestAsdu, uint8_t qoi) {
    printf("Received interrogation for group %i\n", qoi);

    if (qoi == 20) {
        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
        if (serviceConfig == 1) {
            LogRXrequest(qoi);
        }

        IMasterConnection_sendACT_CON(connection, requestAsdu, false);

        // Declare asdus here to ensure it's available in the scope it's used.
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
                    printf("Created ASDU for message type %d\n", msg.messageType);
                }
                for (int j = 0; j < msg.ioContentCount; j++) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);

                    if (io != NULL) {
                        if (!CS101_ASDU_addInformationObject(asdus[msg.messageType], io)) {
                            printf("Failed to add IO (Type %d, IOA %d) to ASDU\n", msg.messageType, ioContent.ioa);
                        } else {
                            printf("Successfully added IO (Type %d, IOA %d) to ASDU\n", msg.messageType, ioContent.ioa);
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

        for (int k = 0; k < MAX_MESSAGE_TYPES; k++) {
            if (asdus[k] && CS101_ASDU_getNumberOfElements(asdus[k]) > 0) {
                IMasterConnection_sendASDU(connection, asdus[k]);
                asduTransmitHandler(asdus[k]);
                printf("Sent ASDU for message type %d with %d elements\n", k, CS101_ASDU_getNumberOfElements(asdus[k]));
            }
            if (asdus[k]) {
                CS101_ASDU_destroy(asdus[k]);  // Clean up after sending
            }
        }
    } else {
        IMasterConnection_sendACT_CON(connection, requestAsdu, true);
    }
    return true;
}

static bool asduHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu) {
    printf("RECVD ASDU type: %s(%i) elements: %i from client\n",
           TypeID_toString(CS101_ASDU_getTypeID(asdu)),
           CS101_ASDU_getTypeID(asdu),
           CS101_ASDU_getNumberOfElements(asdu));

    if (dataConfig == 1) {
        LogRX(CS101_ASDU_getTypeID(asdu), CS101_ASDU_getNumberOfElements(asdu));
    }

    switch (CS101_ASDU_getTypeID(asdu)) {
        case C_SC_NA_1: // 45 Single Command
            printf("  Single command:\n");
            for (int i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {
                SingleCommand io = (SingleCommand) CS101_ASDU_getElement(asdu, i);
                bool value = SingleCommand_getState(io);

                // Correcting the printf to use %s for string output and properly display "true" or "false"
                printf("    IOA: %i value: %s \n",
                       InformationObject_getObjectAddress((InformationObject) io), value ? "true" : "false");

                if (dataConfig == 1) {
                    float logValue = (float) value; // If your LogRXwoT expects a float, convert int to float here
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), logValue);
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
                    case 0: // 0
                        valueDescription = "NOT PERMITTED";
                        break;
                    case 1: // 1
                        valueDescription = "OFF";
                        break;
                    case 2: // 2
                        valueDescription = "ON";
                        break;
                    case 3: // 3
                        valueDescription = "NOT PERMITTED";
                        break;
                    default:
                        valueDescription = "UNKNOWN";
                        break;
                }

                printf("    IOA: %i value: %s\n", InformationObject_getObjectAddress((InformationObject) io), valueDescription);

                if (dataConfig == 1) {
                    float logValue = (float) value; // If your LogRXwoT expects a float, convert int to float here
                    LogRXwoT(InformationObject_getObjectAddress((InformationObject) io), logValue);
                }

                DoubleCommand_destroy(io);
            }
            break;
    }
    return true; // Assuming you need to return a boolean
}



static bool connectionRequestHandler(void *parameter, const char *ipAddress) {
    printf("New connection request from %s\n", ipAddress);
    if (serviceConfig == 1) {
        LogCONREQ(ipAddress);
    }
#if 0
    if (strcmp(ipAddress, "127.0.0.1") == 0) {
    printf("Accept connection\n");
    return true;
  }
  else {
    printf("Deny connection\n");
    return false;
  }
#else
    return true;
#endif
}

static void connectionEventHandler(void *parameter, IMasterConnection con, CS104_PeerConnectionEvent event) {
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("Connection opened (%p)\n", con);
        if (serviceConfig == 1) {
            LogCONOPEN();
        }
    } else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("Connection closed (%p)\n", con);
        if (serviceConfig == 1) {
            LogCONCLOSED();
        }
    } else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);
        if (serviceConfig == 1) {
            LogCONACT();
        }
    } else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("Connection deactivated (%p)\n", con);
        if (serviceConfig == 1) {
            LogCONDEACT();
        }
    }
}

int main(int argc, char **argv) {
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

    lastSentTime = time(NULL);
    char *ip = readConfigValue("KONFIG104SERVER.txt", "IP config");
    char *interface = readConfigValue("KONFIG104SERVER.txt", "Interface");
    char *portStr = readConfigValue("KONFIG104SERVER.txt", "Port");
    char *originatorAddressStr = readConfigValue("KONFIG104SERVER.txt", "Originator Address");
    char *commonAddressStr = readConfigValue("KONFIG104SERVER.txt", "Common Address");
    char *spontaneousConfig = readConfigValue("KONFIG104SERVER.txt", "SPONTANEOUS");
    char *multiplierStr = readConfigValue("KONFIG104SERVER.txt", "MULTI");
    readMessageConfig("KONFIG104SERVER.txt");
    char *dataConfigStr = readConfigValue("KONFIG104SERVER.txt", "DATALOGS");
    dataConfig = atoi(dataConfigStr);
    char *serviceConfigStr = readConfigValue("KONFIG104SERVER.txt", "SERVICELOGS");
    serviceConfig = atoi(serviceConfigStr);
    servicePath = readConfigValue("KONFIG104SERVER.txt", "SERVICEPATH");
    dataPath = readConfigValue("KONFIG104SERVER.txt", "DATAPATH");


    if (serviceConfig == 1) {
        LogSTART();
    }
    FILE *logFile = NULL;

    if (!ip || !interface || !portStr || !originatorAddressStr || !commonAddressStr) {
        fprintf(stderr, "Failed to read some configuration values. Exiting...\n");
        free(ip);
        free(interface);
        free(portStr);
        free(originatorAddressStr);
        free(commonAddressStr);
        return -1;
    }


    if (spontaneousConfig) {
        configureSpontaneousMessages(spontaneousConfig);
        free(spontaneousConfig);
        scheduleNextSpontaneousMessage();
    }

    if (multiplierStr) {
        multiplier = atoi(multiplierStr);
        free(multiplierStr);
    }

    // Načtení konfiguračních hodnot
    char *periodStr = readConfigValue("KONFIG104SERVER.txt", "PERIOD");
    int periodicInterval = periodStr ? atoi(periodStr)
                                     : 20;  // Defaultní perioda je 20 sekund, pokud není specifikováno jinak
    free(periodStr);

    int port = atoi(portStr);
    originatorAddress = atoi(originatorAddressStr);
    commonAddress = atoi(commonAddressStr);

    printf("IP Address: %s\n", ip);
    printf("Interface: %s\n", interface);
    printf("Port: %d\n", port);
    printf("Originator Address: %d\n", originatorAddress);
    printf("Common Address: %d\n", commonAddress);

    CS104_Slave slave = CS104_Slave_create(10, 10);
    CS104_Slave_setLocalAddress(slave, ip);
    CS104_Slave_setLocalPort(slave, port);

    /* Set mode to a single redundancy group
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    /* get the connection parameters - we need them to create correct ASDUs -
     * you can also modify the parameters here when default parameters are not to be used */
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);

    /* when you have to tweak the APCI parameters (t0-t3, k, w) you can access them here */
    CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);

    printf("APCI parameters:\n");
    printf("  t0: %i\n", apciParams->t0);
    printf("  t1: %i\n", apciParams->t1);
    printf("  t2: %i\n", apciParams->t2);
    printf("  t3: %i\n", apciParams->t3);
    printf("  k: %i\n", apciParams->k);
    printf("  w: %i\n", apciParams->w);

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);

    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);


    CS104_Slave_start(slave);


    int16_t scaledValue = 0;

    printf("Server will send messages every %d seconds with multiplier: %d.\n", periodicInterval, multiplier);
    lastSentTime = time(NULL);  // Nastavit čas při startu

    while (running) {
        time_t currentTime = time(NULL);

        // Send periodic messages
        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            printf("Sending periodic messages...\n");

            for (int i = 0; i < numMessageConfigs; ++i) {
                MessageConfig msg = messageConfigs[i];
                CS101_ASDU asdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, originatorAddress, commonAddress, false, false);

                for (int j = 0; j < msg.ioContentCount; ++j) {
                    IOContent ioContent = msg.ioContent[j];
                    InformationObject io = createIO(msg.messageType, ioContent.ioa, ioContent.value);
                    CS101_ASDU_addInformationObject(asdu, io);
                    InformationObject_destroy(io);
                }

                for (int k = 0; k < multiplier; ++k) {
                    CS104_Slave_enqueueASDU(slave, asdu);
                    asduTransmitHandler(asdu);
                    printf("Sent ASDU n. %d periodic with multiplier %d\n", i + 1, k + 1);
                }

                CS101_ASDU_destroy(asdu);
            }

            lastSentTime = currentTime;  // Aktualizace posledního času odeslání
        }

        // Odesílání spontánních zpráv
        if (spontaneousEnabled && currentTime >= nextSpontaneousTime) {
            for (int i = 0; i < multiplier; i++) {
                sendSpontaneousMessage(slave, alParams,
                                       multiplier);  // Now correctly passing all required parameters
            }
            scheduleNextSpontaneousMessage();  // Schedule the next spontaneous message
        }

        Thread_sleep(4000);
    }

    /*CS104_Slave_stop(slave);*/
    CS104_Connection_sendStopDT(slave);
    free(ip);
    free(interface);
    free(portStr);
    free(originatorAddressStr);
    free(commonAddressStr);
    return 0;
    /*exit_program:
      CS104_Slave_destroy(slave);
      Thread_sleep(500)*/
}