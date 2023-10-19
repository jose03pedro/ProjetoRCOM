#include "../include/auxiliary_functions.h"

#include "../include/link_layer.h"

unsigned char txFrame = 0;
unsigned char rxFrame = 1;


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

void stateMachine(unsigned char byte, State *state) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
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

void stateMachineTx(unsigned char byte, State *state) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
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

int stateMachinePck(unsigned char byte, State *state, unsigned char *packet,
                    int fd) {
    int i = 0;
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
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
        case ESC_FOUND:
            *state = READING_DATA;
            if (byte == ESC_FOUND || byte == FLAG)
                packet[i++] = byte;
            else {
                packet[i++] = ESC_FOUND;
                packet[i++] = byte;
            }
            break;
        case READING_DATA:
            if (byte == ESCAPE)
                *state = ESC_FOUND;
            else if (byte == FLAG) {
                unsigned char bcc2 = packet[i - 1];
                i--;
                packet[i] = '\0';
                unsigned char acc = packet[0];

                for (unsigned int j = 1; j < i; j++) acc ^= packet[j];

                if (bcc2 == acc) {
                    *state = STOP_STATE;
                    if (rxFrame == 0) {
                        transmitFrame(fd, A_RS, C_RR0);
                    } else if (rxFrame == 1) {
                        transmitFrame(fd, A_RS, C_RR1);
                    }
                    rxFrame = (rxFrame + 1) % 2;
                    return i;
                } else {
                    printf("Error: retransmition\n");
                    transmitFrame(fd, A_RS, C_REJ(rxFrame));
                    return -1;
                };

            } else {
                packet[i++] = byte;
            }
            break;
        default:
            break;
    }

    return 0;
}

void stateMachineRx(unsigned char byte, State *state) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
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
    if (llwrite(controlPacketStart, cpSize) == -1) {
        printf("Exit: error in start packet\n");
        exit(-1);
    }

    free(controlPacketStart);

    unsigned char* content = getData(file, fileSize);
    long int bytesLeft = fileSize;

    while (bytesLeft > 0) {
        int dataSize = (bytesLeft > MAX_PAYLOAD_SIZE) ? MAX_PAYLOAD_SIZE : bytesLeft;
        unsigned char* data = (unsigned char*) malloc(dataSize);
        memcpy(data, content, dataSize);
        int packetSize;
        unsigned char* packet = getDataPacket(sequence, data, dataSize, &packetSize);

        if (llwrite(packet, packetSize) == -1) {
            printf("Exit: error in data packets\n");
            exit(-1);
        }

        free(data);
        free(packet);

        bytesLeft -= MAX_PAYLOAD_SIZE;
        content += dataSize;
        sequence = (sequence + 1) % 255;
    }

    unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &cpSize);
    if (llwrite(controlPacketEnd, cpSize) == -1) {
        printf("Exit: error in end packet\n");
        exit(-1);
    }

    free(controlPacketEnd);
}

int createFrame(unsigned char **frame, const unsigned char *buf, int bufSize) {
    int frameSize = 6 + bufSize;
    *frame = (unsigned char *)malloc(frameSize);
    if (*frame == NULL) {
        return -1; 
    }

    (*frame)[0] = FLAG;
    (*frame)[1] = A_SR;
    (*frame)[2] = C_NS(rxFrame);
    (*frame)[3] = (*frame)[1] ^ (*frame)[2];
    memcpy(*frame + 4, buf, bufSize);

    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1; i < bufSize; i++) {
        BCC2 ^= buf[i];
    }

    int j = 4;
    for (unsigned int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG || buf[i] == ESCAPE) {
            (*frame) = (unsigned char *)realloc(*frame, ++frameSize);
            if (*frame == NULL) {
                return -1;
            }
            (*frame)[j++] = ESCAPE;
        }
        (*frame)[j++] = buf[i];
    }
    (*frame)[j++] = BCC2;
    (*frame)[j++] = FLAG;

    return frameSize;
}

int sendFrame(int fd, const unsigned char *frame, int frameSize, int *retransmissions, int timer, int *alarmEnabled) {
    int currentTransmission = 0;

    while (currentTransmission < retransmissions) {
        alarmEnabled = FALSE;
        alarm(timer);

        if (write(fd, frame, frameSize) == -1) {
            free(frame);
            return -1;
        }

        unsigned char result = readControlFrame(fd);

        if (!result) {
            continue;
        } else if (result == C_REJ0 || result == C_REJ1) {
            currentTransmission++;
        } else if (result == C_RR0 || result == C_RR1) {
            rxFrame = (rxFrame + 1) % 2;
            free(frame);
            return frameSize;
        } else {
            continue;
        }
    }

    free(frame);
    return -1;
}