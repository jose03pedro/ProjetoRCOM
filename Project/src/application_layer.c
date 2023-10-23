
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

    printf("openconnection fine\n");
    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
        printf("before bufsize inside tx\n");
        printf("before bufsize after tx\n");

        if (llopen(connectionParameters) == -1) {
            perror("Error opening connection\n");
            exit(-1);
        }
        printf("before fopen after llopen\n");
        FILE* file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }
        printf("before bufsize\n");
        int temp = ftell(file);
        fseek(file, 0L, SEEK_END);
        long int fileSize = ftell(file) - temp;
        fseek(file, temp, SEEK_SET);
        int controlPacketSize;
        unsigned char* controlPacket;
        controlPacket = buildControlPacket(2, fileSize, filename, &controlPacketSize);
        printf("before llwrite\n");
        if (llwrite(controlPacket, controlPacketSize) < 0) {
            perror("Error sending control packet\n");
            exit(-1);
        }
        printf("after llwrite\n");

        unsigned char* content;
        int dataSize;
        content = getData(file, fileSize);
        long int bytesLeft = fileSize;

        while (bytesLeft > 0) {
            if (bytesLeft > (long int) MAX_PAYLOAD_SIZE) {
                dataSize = MAX_PAYLOAD_SIZE;
            } else {
                dataSize = bytesLeft;
            }

            unsigned char* data;
            data = (unsigned char*) malloc(dataSize);
            memcpy(data, content, dataSize);

            int datapSize;
            unsigned char *dataPacket;
            dataPacket = buildDataPacket(data, dataSize, &datapSize);
            if (llwrite(dataPacket, datapSize) == -1) {
                perror("Error sending data packet\n");
                exit(-1);
            }
            bytesLeft -= dataSize;
            data += dataSize;
        }
        int controlPacketEndSize;
        unsigned char* controlPacketEnd;
        controlPacketEnd = buildControlPacket(3, fileSize, filename, &controlPacketEndSize);
        if (llwrite(controlPacketEnd, controlPacketEndSize) == -1) {
            perror("Error sending control packet\n");
            exit(-1);
        }
        llclose(1);
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;

        if (llopen(connectionParameters) == -1) {
            perror("Error opening connection\n");
            exit(-1);
        }
        FILE* file = fopen((char*) filename, "wb+");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }
        unsigned char *packet;
        int pSize = -1;
        packet = (unsigned char *) malloc(MAX_PAYLOAD_SIZE + 1 + 9);
        unsigned long int fileSize = 0;
        unsigned char *fileName = processControlPacket(packet, pSize, &fileSize);
        while(1){
            while ((pSize = llread(packet)) < 0);
            if (packet[0] == 2) {
                file = fopen((char*) fileName, "wb+");
            } else if (packet[0] == 1) {
                unsigned char *buf = (unsigned char *) malloc(pSize);
                memcpy(buf, packet + 3, pSize - 3);
                fwrite(buf, sizeof(unsigned char), pSize - 3, file);
                free(buf);
            }
            else if(packet[0] == 3){
                printf("End of transmission\n");
                break;
            }
            else{
                printf("Invalid packet\n");
                break;            
            }
            free(packet);
        }
        fclose(file);
        llclose(1);  
    }
    else {
        perror("Invalid role\n");
        exit(-1);
    }
}


unsigned char* buildControlPacket(unsigned int controlFieldValue, const long int fileSize, const char * filename, int *cpSize){

    //Size in bytes of the filename (each char is 1 byte long)
    int L2 = strlen(filename);

    //Size in bytes of the fileSize
    long int tempSize = fileSize;
    int L1 = 0;
    while(tempSize != 0){
        tempSize  = tempSize >> 8;
        L1++;
    }

    *cpSize = 1 + 2 + L1 + 2 + L2;
    unsigned char *controlPacket = (unsigned char*) malloc(*cpSize);

    unsigned int offset = 0;

    controlPacket[offset++] = controlFieldValue;
    controlPacket[offset++] = 0;
    controlPacket[offset++] = L1;

    long int temp = fileSize;
    for(unsigned char j = 0; j < L1; j++){
        controlPacket[2 + L1 - j] = temp & 0xFF;
        temp = temp >> 8;
    }

    //Comprimento de v1 em bytes é o l1

    offset += L1;
    controlPacket[offset++] = 1;
    controlPacket[offset++] = L2;
    memcpy(controlPacket + offset, filename, L2);

    return controlPacket;

}

unsigned char* getData (FILE* file, long int filesize){

    unsigned char* data = (unsigned char*) malloc(filesize);
    fread(data, filesize, sizeof(unsigned char), file);
    return data;

}

unsigned char* buildDataPacket(unsigned char* data, int dataSize, int *pSize){

    *pSize = 1 + 2 + dataSize;
    unsigned char * dataPacket = (unsigned char*) malloc(dataSize);
    dataPacket[0] = 1;
    dataPacket[1] = (dataSize >> 8) & 0xFF;
    dataPacket[2] = dataSize & 0xFF;

    memcpy(dataPacket + 3, data, dataSize);
    return dataPacket;

}

unsigned char* processControlPacket(unsigned char *packet, int readSize, unsigned long int *fileSize){

    unsigned char fileSizeInBytes = packet[2];

    unsigned char sizeAux[fileSizeInBytes];

    memcpy(sizeAux, packet+3, fileSizeInBytes);

    for( unsigned int i = 0; i < fileSizeInBytes; i++){

        *fileSize |= (sizeAux[fileSizeInBytes-i-1] << (8*i));
    }

    unsigned char nameSizeInBytes = packet[3 + fileSizeInBytes + 1];

    unsigned char *nameAux = (unsigned char*) malloc(nameSizeInBytes);
    
    memcpy(nameAux, packet+3+fileSizeInBytes+2, nameSizeInBytes);

    return nameAux;
}