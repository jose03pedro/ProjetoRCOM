
// Application layer protocol implementation

#include "../include/application_layer.h"

#include "../include/link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    strcpy(connectionParameters.serialPort, serialPort);

    fd = openConnection(connectionParameters.serialPort);

    printf("fd1: %d\n", fd);
    printf("openconnection fine\n");
    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
        printf("before bufsize inside tx\n");
        // int fd = openConnection(serialPort);
        printf("fd2: %d\n", fd);
        printf("before bufsize after tx\n");

        if (llopen(connectionParameters) == -1) {
            perror("Error opening connection\n");
            exit(-1);
        }
        printf("before fopen after llopen\n");
        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }
        printf("before bufsize\n");
        const int bufSize = MAX_PAYLOAD_SIZE;
        unsigned char buffer[bufSize + 1];  // primeiro byte para dizer se deve
                                            // continuar(1) ou parar(0)
        int writeResult = 0;                // output do llwrite
        int bytesRead = 1;  // entrar no loop, controlar n de bytes lidos
        while (bytesRead > 0) {
            bytesRead = read(fd, buffer + 1, bufSize);
            if (bytesRead < 0) {
                perror("Error receiving from link layer\n");
                exit(-1);
                break;
            } else if (bytesRead > 0) {
                buffer[0] = 1;  // byte da transmissao (continuar)
                writeResult = llwrite(buffer, bytesRead + 1);
                if (writeResult < 0) {
                    perror("Error sending data to link layer\n");
                    exit(-1);
                    break;
                }
                printf("read from file -> write to link layer, %d\n",
                       bytesRead);
            } else if (bytesRead == 0) {
                buffer[0] = 0;  // parar
                llwrite(buffer, 1);
                printf("App layer: done reading and sending file\n");
                break;
            }

            sleep(1);
        }
        // close connection
        llclose(fd);
    } else if (strcmp(role, "rx") == 0) {
        int writeResult = 0;
        int totalBytes = 0;
        connectionParameters.role = LlRx;
        // fd = openConnection(serialPort);
        printf("fd2: %d\n", fd);

        if (llopen(connectionParameters) == -1) {
            perror("Error opening connection\n");
            exit(-1);
        }
        printf("fdll: %d\n", fd);
        FILE *file = fopen(filename, "wb");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }
        printf("fdn: %d\n", fd);
        const int bufSize = MAX_PAYLOAD_SIZE;
        unsigned char buffer[bufSize];
        int bytesRead = 1;

        while (1) {
            printf("bytesaa %d\n", bytesRead);
            bytesRead = llread(buffer);
            printf("bytesaa2 %d\n", bytesRead);
            if (bytesRead < 0) {
                perror("Error receiving from link layer\n");
                exit(-1);
                break;
            } else if (bytesRead > 0) {
                // Escrever os bytes recebidos no arquivo
                writeResult =
                    fwrite(buffer, sizeof(unsigned char), bytesRead, file);
                if (writeResult < 0) {
                    perror("Error writing to file\n");
                    exit(-1);
                    break;
                }
                totalBytes += writeResult;
            } else if (bytesRead == 0) {
                buffer[0] = 0;
                llwrite(buffer, 1);
                printf("App layer: Recepetion Complete\n");
                break;
            }
        }

        fclose(file);
        llclose(fd);
    }

    else {
        perror("Invalid role\n");
        exit(-1);
    }
}