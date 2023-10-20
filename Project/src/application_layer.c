
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

        const int buf_size = MAX_PAYLOAD_SIZE;
        unsigned char buffer[buf_size + 1];  // primeiro byte para dizer se deve
                                             // continuar(1) ou parar(0)
        int write_result = 0;                // output do llwrite
        int bytes_read = 1;  // entrar no loop, controlar n de bytes lidos
        while (bytes_read > 0) {
            bytes_read = read(fd, buffer + 1, buf_size);
            if (bytes_read < 0) {
                perror("Error receiving from link layer\n");
                exit(-1);
                break;
            } else if (bytes_read > 0) {
                buffer[0] = 1;  // byte da transmissao (continuar)
                write_result = llwrite(buffer, bytes_read + 1);
                if (write_result < 0) {
                    perror("Error sending data to link layer\n");
                    exit(-1);
                    break;
                }
                printf("read from file -> write to link layer, %d\n",
                       bytes_read);
            } else if (bytes_read == 0) {
                buffer[0] = 0;  // parar
                llwrite(buffer, 1);
                printf("App layer: done reading and sending file\n");
                break;
            }

            sleep(1);
        }
        // close connection
        llclose(fd);

        /*
                    int fileSizePrev = ftell(file);
                    long int zero = 0;
                    fseek(file, zero, SEEK_END);
                    long int fileSize = ftell(file) - fileSizePrev;
                    fseek(file, fileSizePrev, SEEK_SET);

                    sendControlPackets(fd, filename, fileSize, 0);


                    llclose(fd);
        */

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
        while (packetSize < 0)
            ;
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
        free(packet);
    } else {
        perror("Invalid role\n");
        exit(-1);
    }
}