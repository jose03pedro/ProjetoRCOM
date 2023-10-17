// Link layer protocol implementation

#include "../include/link_layer.h"
#include "../include/macros.h"

int alarmCount = 0;
int alarmEnabled = FALSE;
int retransmissions = 0;
int timer = 0;
unsigned char tx_frame = 0;
unsigned char rx_frame = 1;
volatile int STOP = FALSE;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    State state = START;
    int fd = openConnection(connectionParameters.serialPort);
    if (fd < 0) return -1;

    unsigned char byte;
    timer = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;
    switch (connectionParameters.role) {

        case LlTx: {
            (void) signal(SIGALRM, alarmHandler);

            do {
                sendSFrame(fd, A_SR, C_SET);
                alarm(timer);
                alarmEnabled = FALSE;

                do {
                    switch (read(fd, &byte, 1)) {
                        case 1:
                            stateMachineTx(byte, &state);
                            if(state == STOP_STATE) {
                                printf("Connection established\n");
                            }
                            break;
                        case -1:
                            return -1;
                            break;
                        default:
                            break;
                    }
                } while (alarmEnabled == FALSE && state != STOP_STATE);

                retransmissions--;
            } while (retransmissions != 0 && state != STOP_STATE);

            if (state != STOP_STATE) return -1;
            break;
        }

        case LlRx: {
            do {
                switch (read(fd, &byte, 1)) {
                    case 1:
                        stateMachineRx(byte, &state);
                        if(state == STOP_STATE) {
                            printf("Connection established\n");
                        }
                        break;
                    case -1:
                        return -1;
                        break;
                    default:
                        break;
                }
            } while (state != STOP_STATE);

            sendSFrame(fd, A_RS, C_UA);
            break;
        }

        default:
            return -1;
            break;
    }

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}

////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////

int sendSFrame(int fd, unsigned char A, unsigned char C) {
    unsigned char f[5];
    f[0] = FLAG;
    f[1] = A;
    f[2] = C;
    f[3] = BCC(A, C);
    f[4] = FLAG;

    return write(fd, f, sizeof(f));
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