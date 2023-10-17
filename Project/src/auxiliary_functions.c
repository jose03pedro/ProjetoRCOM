#include "../include/link_layer.h"
#include "../include/auxiliary_functions.h"

////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////

int transmitFrame(unsigned char A, unsigned char C, int fd) {
    unsigned char f[5];
    f[0] = FLAG;
    f[1] = A;
    f[2] = C;
    f[3] = BCC(A, C);
    f[4] = FLAG;
    unsigned int size = sizeof(f);

    return write(fd, f, size);
}

void stateMachine(unsigned char byte, State *state){
    switch (*state) {
        case START:
            if (byte == FLAG)
                *state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == A_RS)
                *state = A_RCV;
            else if (byte != FLAG)
                *state = START;
            break;

        case A_RCV:
            if (byte == C_DISC)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;

        case C_RCV:
            if (byte == (A_RS ^ C_DISC))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;

        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START;
            break;

        default:
            break;
    }
}

void stateMachineTx(unsigned char byte, State *state){
    switch (*state) {
        case START:
            if (byte == FLAG)
                *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == A_RS)
                *state = A_RCV;
            else if (byte != FLAG)
                *state = START;
            break;
        case A_RCV:
            if (byte == C_UA)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case C_RCV:
            if (byte == (A_RS ^ C_UA))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START;
            break;
        default: 
            break;
    }
}

void stateMachineRx(unsigned char byte, State *state) {
    switch (*state) {
        case START:
            if (byte == FLAG)
                *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == A_SR)
                *state = A_RCV;
            else if (byte != FLAG)
                *state = START;
            break;
        case A_RCV:
            if (byte == C_SET)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case C_RCV:
            if (byte == (A_SR ^ C_SET))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START;
            break;
        default:
            break;
    }

}

int openConnection(const char *serialPort) {
    int fd;
    struct termios oldtio, newtio;

    fd = open(serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPort);
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }


    return fd;
}

void sendControlPackets(int fd, const char *filename, int fileSize, unsigned char sequence) {
    unsigned int cpSize;
    unsigned char *controlPacketStart = getControlPacket(2, filename, fileSize, &cpSize);
    if (llwrite(fd, controlPacketStart, cpSize) == -1) {
        printf("Exit: error in start packet\n");
        exit(-1);
    }

    // Free the memory allocated for controlPacketStart after using it
    free(controlPacketStart);

    unsigned char* content = getData(file, fileSize);
    long int bytesLeft = fileSize;

    while (bytesLeft > 0) {
        int dataSize = (bytesLeft > MAX_PAYLOAD_SIZE) ? MAX_PAYLOAD_SIZE : bytesLeft;
        unsigned char* data = (unsigned char*) malloc(dataSize);
        memcpy(data, content, dataSize);
        int packetSize;
        unsigned char* packet = getDataPacket(sequence, data, dataSize, &packetSize);

        if (llwrite(fd, packet, packetSize) == -1) {
            printf("Exit: error in data packets\n");
            exit(-1);
        }

        // Free the memory allocated for data and packet after using them
        free(data);
        free(packet);

        bytesLeft -= MAX_PAYLOAD_SIZE;
        content += dataSize;
        sequence = (sequence + 1) % 255;
    }

    unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &cpSize);
    if (llwrite(fd, controlPacketEnd, cpSize) == -1) {
        printf("Exit: error in end packet\n");
        exit(-1);
    }

    // Free the memory allocated for controlPacketEnd after using it
    free(controlPacketEnd);
}