
// Application layer protocol implementation

#include "../include/application_layer.h"

#include "../include/link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    strcpy(connectionParameters.serialPort, serialPort);
    printf("role: %d\n", connectionParameters.role);
    printf("serialPort: %s\n", connectionParameters.serialPort);
    printf("baudRate: %d\n", connectionParameters.baudRate);
    printf("nRetransmissions: %d\n", connectionParameters.nRetransmissions);
    printf("timeout: %d\n", connectionParameters.timeout);
    llopen(connectionParameters);
    switch (connectionParameters.role){
    case LlTx: {
        FILE* file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Error opening file\n");
            exit(-1);
        }
        printf("aaaaaafsfas\n");
        int pos = ftell(file);  // curr file position indicator to the startpos
        fseek(file, 0L, SEEK_END);  // offset 0 bytes, move file pointer to EOF
        long int fileSize = ftell(file) - pos;  // EOF - Startpos = size of the file
        fseek(file, pos, SEEK_SET);  // reposiciona o pointer para a posição original
        int controlPacketSize;
        unsigned char *controlPacket;
        controlPacket = buildControlPacket(2, fileSize, filename, &controlPacketSize);
        printf("before llwrite\n");
        if (llwrite(controlPacket, controlPacketSize) < 0) {
            perror("Error sending control packet\n");
            exit(-1);
        }
        printf("after llwrite\n");

        unsigned char *content = (unsigned char *)malloc(fileSize);
        fread(content, fileSize, sizeof(unsigned char), file);
        long int bytesLeft = fileSize;

        while (bytesLeft > 0) {
            int dataSize;
            if (bytesLeft > (long int)MAX_PAYLOAD_SIZE / 2) {
                dataSize = MAX_PAYLOAD_SIZE / 2;
            } else {
                dataSize = bytesLeft;
            }

            unsigned char *data = (unsigned char *)malloc(dataSize);
            memcpy(data, content, dataSize);

            int datapSize = 1 + 2 + dataSize;
            unsigned char *dataPacket = (unsigned char *)malloc(dataSize);
            dataPacket[0] = DATA_PAYLOAD;
            dataPacket[1] = (dataSize >> 8) & 0xFF;
            dataPacket[2] = dataSize & 0xFF;

            memcpy(dataPacket + 3, data, dataSize);

            if (llwrite(dataPacket, datapSize) == -1) {
                perror("Error sending data packet\n");
                exit(-1);
            }
            bytesLeft -= dataSize;
            data += dataSize;
        }
        int controlPacketEndSize;
        unsigned char *controlPacketEnd;
        controlPacketEnd = buildControlPacket(3, fileSize, filename, &controlPacketEndSize);
        if (llwrite(controlPacketEnd, controlPacketEndSize) == -1) {
            perror("Error sending control packet\n");
            exit(-1);
        }
        if (llclose(1) == -1) {
            perror("Error closing connection\n");
            exit(-1);
        }
        printf("Connection closed\n");
        break;
    }
        
    case LlRx: {
        FILE* file;
    
        while (1) {
            unsigned char *packet;
            int pSize = -1;
            packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE + 1 + 9);
            while ((pSize = llread(packet)) < 0);
            if (packet[0] == 2) {
                unsigned long int fileSize = 0;
                unsigned char *fileName = processControlPacket(packet, pSize, &fileSize);
                file = fopen((char *)fileName, "wb+");
            } else if (packet[0] == 1) {
                unsigned char *buf = (unsigned char *)malloc(pSize);
                memcpy(buf, packet + 3, pSize - 3);
                fwrite(buf, sizeof(unsigned char), pSize - 3, file);
                free(buf);
            } else if (packet[0] == 3) {
                printf("End of transmission\n");
                break;
            } else {
                printf("Invalid packet\n");
                break;
            }
            free(packet);
        }
        fclose(file);

        printf("File received\n");
        llclose(1);
        printf("Connection closed\n");
    }
    default:
        perror("Invalid role\n");
        exit(-1);
        break;
    }
}

unsigned char *buildControlPacket(unsigned int controlFieldValue,
                                  const long int fileSize, const char *filename,
                                  int *packetSize) {
    // Calculate the size in bytes of the filename (assuming each char is 1
    // byte)
    int filenameSizeBytes = strlen(filename);

    // Calculate the size in bytes needed for the fileSize field
    long int tempFileSize = fileSize;
    int fileSizeSizeBytes = 0;
    while (tempFileSize != 0) {
        tempFileSize = tempFileSize >> 8;
        fileSizeSizeBytes++;
    }

    // Calculate the total size of the control packet
    *packetSize = 1 + 2 + fileSizeSizeBytes + 2 + filenameSizeBytes;

    // Allocate memory for the control packet
    unsigned char *controlPacket = (unsigned char *)malloc(*packetSize);

    // Initialize offset to keep track of the current position in the control
    // packet
    unsigned int offset = 0;

    // Add the control field value to the control packet
    controlPacket[offset++] = controlFieldValue;

    // Add reserved byte (0) to the control packet
    controlPacket[offset++] = 0;

    // Add the fileSize size in bytes to the control packet
    controlPacket[offset++] = fileSizeSizeBytes;

    // Copy the fileSize bytes to the control packet
    long int temp = fileSize;
    for (unsigned char j = 0; j < fileSizeSizeBytes; j++) {
        controlPacket[2 + fileSizeSizeBytes - j] = temp & 0xFF;
        temp = temp >> 8;
    }

    // Move the offset to the next field
    offset += fileSizeSizeBytes;

    // Add the filename field indicator (1) to the control packet
    controlPacket[offset++] = 1;

    // Add the size of the filename field to the control packet
    controlPacket[offset++] = filenameSizeBytes;

    // Copy the filename to the control packet
    memcpy(controlPacket + offset, filename, filenameSizeBytes);

    return controlPacket;
}

unsigned char *processControlPacket(unsigned char *packet, int readSize,
                                    unsigned long int *fileSize) {
    unsigned char fileSBytes = packet[2];
    unsigned char size[fileSBytes];

    memcpy(size, packet + 3, fileSBytes);

    unsigned int shift = 0;
    for (unsigned int i = 0; i < fileSBytes; i++) {
        *fileSize |= (size[fileSBytes - i - 1] << (8 * (shift++)));
    }

    unsigned char nameSBytes = packet[3 + fileSBytes + 1];
    unsigned char *nameAux = (unsigned char *)malloc(nameSBytes);

    memcpy(nameAux, packet + 3 + fileSBytes + 2, nameSBytes);

    return nameAux;
}