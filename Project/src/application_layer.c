// Application layer protocol implementation

#include "../include/application_layer.h"

#include "../include/auxiliary_functions.h"
#include "../include/link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    strcpy(connectionParameters.serialPort, serialPort);

    int fd = openConnection(serialPort);

    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
        fd = openConnection(serialPort);

        if (llopen(connectionParameters) == -1) {
            perror("Error opening connection\n");
            exit(-1);
        }

        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }

        int fileSizePrev = ftell(file);
        long int zero = 0;
        fseek(file, zero, SEEK_END);
        long int fileSize = ftell(file) - fileSizePrev;
        fseek(file, fileSizePrev, SEEK_SET);

        sendControlPackets(fd, filename, fileSize, 0);

        llclose(fd);
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
        fd = openConnection(serialPort);

        unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        int packetSize = llread(packet);  // verificar o valor do packetSize
                                          // pois pode não estar a ser alterado
        if (packetSize == -1) {
            perror("Error reading packet\n");
            exit(-1);
        }
        // while (packetSize < 0);
        unsigned long int rxFileSize = 0;
        unsigned char *name = parseControlPacket(
            packet, packetSize,
            &rxFileSize);  // função a ser implementada e trocar um bocado a
                           // variaveis e assim (plagio)
        FILE *newFile = fopen((char *)name, "wb+");
        while (1) {
            int packetSize = llread(packet);
            if (packetSize == -1) {
                perror("Error reading packet\n");
                exit(-1);
            }
            while (packetSize < 0)
                ;
            if (packetSize == 0)
                break;
            else if (packet[0] != 3) {
                unsigned char *buffer = (unsigned char *)malloc(packetSize);
                parseDataPacket(
                    packet, packetSize,
                    buffer);  // função a ser implementada e trocar um bocado a
                              // variaveis e assim (plagio)
                fwrite(buffer, sizeof(unsigned char), packetSize - 4, newFile);
                free(buffer);
            } else
                continue;
        }
        fclose(newFile);
    } else {
        perror("Invalid role\n");
        exit(-1);
    }
}