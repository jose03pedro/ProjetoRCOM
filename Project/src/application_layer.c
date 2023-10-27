#include "../include/application_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    strcpy(connectionParameters.serialPort, serialPort);

    llopen(connectionParameters);

    LinkLayerRole connection_role = connectionParameters.role;
    FILE *file = NULL;

    switch (connection_role) {
        case LlTx:
            file = fopen(filename, "rb");
            if (file == NULL) {
                fprintf(stderr, "Error: Failed to open the file\n");
                exit(-1);
            }

            long int fileSize = getFileSize(file);

            if (sendControlPacket(START, fileSize, filename) == -1) {
                fprintf(stderr, "Error: Failed to send control packet\n");
                exit(-1);
            }

            unsigned char *content = (unsigned char *)malloc(fileSize);
            fread(content, fileSize, sizeof(unsigned char), file);

            while (fileSize > 0) {
                int dataSize = (fileSize > (long int)(MAX_PAYLOAD_SIZE / 2))
                                   ? MAX_PAYLOAD_SIZE / 2
                                   : fileSize;

                if (sendDataPacket(dataSize, content) == -1) {
                    fprintf(stderr, "Error: Failed to send data packet\n");
                    exit(-1);
                }

                fileSize -= dataSize;
                content += dataSize;
            }

            if (sendControlPacket(END, fileSize, filename) == -1) {
                fprintf(stderr, "Error: Failed to send end control packet\n");
                exit(-1);
            }

            if (llclose(1) == -1) {
                fprintf(stderr, "Error: Failed to close the connection\n");
                exit(-1);
            }

            printf("Connection closed\n");
            break;

        case LlRx:
            file = receiveFile(filename);

            if (llclose(1) == -1) {
                fprintf(stderr, "Error: Failed to close the connection\n");
                exit(-1);
            }

            printf("Connection closed\n");
            break;

        default:
            fprintf(stderr, "Error: Invalid role\n");
            exit(-1);
            break;
    }
}

long int getFileSize(FILE *file) {
    int start_pos = ftell(file);
    fseek(file, 0L, SEEK_END);
    long int fileSize = ftell(file) - start_pos;
    fseek(file, start_pos, SEEK_SET);
    return fileSize;
}

int sendControlPacket(unsigned char controlField, long int fileSize,
                      const char *filename) {
    int controlPacketSize;
    unsigned char *controlPacket = buildControlPacket(
        controlField, fileSize, filename, &controlPacketSize);
    return llwrite(controlPacket, controlPacketSize);
}

int sendDataPacket(int dataSize, unsigned char *content) {
    int datapSize = 1 + 2 + dataSize;
    unsigned char *dataPacket = (unsigned char *)malloc(datapSize);
    dataPacket[0] = 1;
    dataPacket[1] = (dataSize >> 8) & 0xFF;
    dataPacket[2] = dataSize & 0xFF;
    memcpy(dataPacket + 3, content, dataSize);
    int result = llwrite(dataPacket, datapSize);
    free(dataPacket);
    return result;
}

FILE *receiveFile(const char *filename) {
    FILE *file = fopen(filename, "wb+");
    while (1) {
        int pSize = -1;
        unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        while (pSize < 0) {
            pSize = llread(packet);
        }
        if (packet[0] == 2) {
            printf("First llread\n");
            unsigned long int newfsize = 0;
            processControlPacket(packet, pSize, &newfsize);
        } else if (packet[0] == 1) {
            unsigned char *buf = (unsigned char *)malloc(pSize);
            memcpy(buf, packet + 3, pSize - 3);
            fwrite(buf, 1, pSize - 3, file);
            free(buf);
        } else if (packet[0] == 3) {
            break;
        } else {
            fprintf(stderr, "Error: Invalid packet\n");
            break;
        }
        free(packet);
    }
    return file;
}

unsigned char *buildControlPacket(unsigned int controlField,
                                  const long int fileSize, const char *filename,
                                  int *packetSize) {
    int filenameLength = strlen(filename);
    int fileSizeLength = 0;
    long int tempFileSize = fileSize;

    // Calculate the number of bytes needed to represent the fileSize
    while (tempFileSize != 0) {
        tempFileSize = tempFileSize >> 8;
        fileSizeLength++;
    }

    // Calculate the total size of the control packet
    *packetSize = 1 + 2 + fileSizeLength + 2 + filenameLength;

    // Allocate memory for the control packet
    unsigned char *controlPacket = (unsigned char *)malloc(*packetSize);

    unsigned int offset = 0;

    // Add the control field to the control packet
    controlPacket[offset++] = controlField;

    // Add reserved byte (0) to the control packet
    controlPacket[offset++] = 0;

    // Add the fileSize size in bytes to the control packet
    controlPacket[offset++] = fileSizeLength;

    // Copy the fileSize bytes to the control packet
    tempFileSize = fileSize;
    for (int i = 0; i < fileSizeLength; i++) {
        controlPacket[offset + fileSizeLength - i] = tempFileSize & 0xFF;
        tempFileSize = tempFileSize >> 8;
    }

    offset += fileSizeLength;

    // Add the filename field indicator
    controlPacket[offset++] = 1;

    // Add the size of the filename field to the control packet
    controlPacket[offset++] = filenameLength;

    // Copy the filename to the control packet
    memcpy(controlPacket + offset, filename, filenameLength);

    return controlPacket;
}

unsigned char *processControlPacket(unsigned char *packet, int readSize,
                                    unsigned long int *fileSize) {
    unsigned char sizeLen = packet[2];
    unsigned char sizeBytes[sizeLen];
    memcpy(sizeBytes, packet + 3, sizeLen);

    *fileSize = 0;

    for (int i = 0; i < sizeLen; i++) {
        *fileSize |=
            ((unsigned long int)sizeBytes[i] << (8 * (sizeLen - i - 1)));
    }

    unsigned char nameLen = packet[3 + sizeLen + 1];
    unsigned char *filename = (unsigned char *)malloc(nameLen);

    memcpy(filename, packet + 3 + sizeLen + 2, nameLen);

    return filename;
}
