// Link layer protocol implementation

#include "../include/link_layer.h"

#include "../include/auxiliary_functions.h"

int alarmCount = 0;
int alarmEnabled = FALSE;
const char *serialPort;



void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
}

int retransmissions = 0;
int timer = 0;
unsigned char tx_frame = 0;
unsigned char rx_frame = 1;
volatile int STOP = FALSE;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    State state = START;
    int fd;
    if ((fd = openConnection(connectionParameters.serialPort)) < 0) {
        return -1;
    }

    LinkLayerRole role = connectionParameters.role;

    unsigned char rByte;
    timer = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;
    serialPort = connectionParameters.serialPort;

    int retransmissions_var = retransmissions;

    if (role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);

        do {
            transmitFrame(fd, A_SR, C_SET); // send SET frame
            alarm(timer);
            alarmEnabled = FALSE;

            do {
                switch (read(fd, &rByte, 1)) {
                    case 1:
                        stateMachineTx(rByte, &state);
                        if (state == STOP_STATE) {
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

            retransmissions_var--;
        } while (retransmissions_var != 0 && state != STOP_STATE);

        if (state != STOP_STATE) return -1;
    } else if (role == LlRx) {
        do {
            switch (read(fd, &rByte, 1)) {
                case 1:
                    stateMachineRx(rByte, &state);
                    if (state == STOP_STATE) {
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

        transmitFrame(fd, A_RS, C_UA); // send UA frame
    } else {
        return -1;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    int fd = openConnection(serialPort);

    // Create frame
    unsigned char frame[MAX_PAYLOAD_SIZE] = {0};
    frame[0] = FLAG;
    frame[1] = A_SR;
    if (txFrame == 0) {
        frame[2] = C_I0;
    } else {
        frame[2] = C_I1;
    }
    frame[3] = BCC(frame[1], frame[2]);

    int bsize = bufSize;
    unsigned char bcc2 = 0;
    bcc2 = buf[0];
    int var = 0;

    // BCC2
    while (bsize > 0)
    {
        var++;
        bcc2 ^= buf[var];
        bsize--;
    }

    // Stuffing
    int i = 4;
    int j = 0;
    unsigned char value = 0;
    while(j < bufSize)
    {
        value = buf[j];
        switch (value)
        {
        case FLAG:
            frame[i] = ESCAPE;
            frame[i+1] = ESCAPE_FLAG;
            i+=2;
            break;
        case ESCAPE:
            frame[i] = ESCAPE;
            frame[i+1] = ESCAPE_ESCAPE;
            i+=2;
            break;
        default:
            frame[i] = buf[j];
            i++;
            break;
        }
        j++;
    }

    // BCC2 Stuffing
    switch (bcc2)
    {
    case FLAG:
        frame[i] = ESCAPE;
        frame[i+1] = ESCAPE_FLAG;
        i+=2;
        break;
    case ESCAPE:
        frame[i] = ESCAPE;
        frame[i+1] = ESCAPE_ESCAPE;
        i+=2;
        break;
    default:
        frame[i] = bcc2;
        i++;
        break;
    }

    // End of frame
    frame[i] = FLAG;

    int retransmissions_var = retransmissions;

    State state = START;
    (void)signal(SIGALRM, alarmHandler);
    unsigned char rByte_temp = 0;

    // Send frame
    while (retransmissions_var > 0)
    {
        write(fd, frame, i); // i = frame size
        alarm(timer);
        alarmEnabled = TRUE;

        while (STOP == FALSE && alarmEnabled == TRUE)
        {
            unsigned char rByte;
            read(fd, &rByte, 1);
            while (state != STOP_STATE)
            {
                switch (state)
                {
                case START:
                    if (rByte == FLAG)
                    {
                        state = FLAG_RCV;
                    } 
                    break;
                case FLAG_RCV:
                    if (rByte == A_SR)
                    {
                        state = A_RCV;
                    } else if (rByte != FLAG)
                    {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (rByte == C_RR0 || rByte == C_RR1 || rByte == C_REJ0 || rByte == C_REJ1)
                    {
                        state = C_RCV;
                        rByte_temp = rByte;
                    } else if (rByte == FLAG)
                    {
                        state = FLAG_RCV;
                    } else
                    {
                        state = START;
                    }
                    break;

                case C_RCV:
                    if (rByte == BCC(A_SR, C_I0) || rByte == BCC(A_SR, C_I1))
                    {
                        state = BCC1_OK;
                    } else if (rByte == FLAG)
                    {
                        state = FLAG_RCV;
                    } else
                    {
                        state = START;
                    }
                    break;
                case BCC1_OK:
                    if (rByte == FLAG)
                    {
                        state = STOP_STATE;
                    } else
                    {
                        state = START;
                    }
                    break;
                case STOP_STATE:
                    if (rByte == FLAG)
                    {
                        if(rByte_temp == C_RR0 || rByte_temp == C_RR1)
                        {
                            if(rByte_temp == C_RR0)
                            {
                                txFrame = 1;
                                rxFrame = 0;
                            } else
                            {
                                txFrame = 0;
                                rxFrame = 1;
                            }
                            STOP = TRUE;
                            retransmissions_var = 0;
                        } else if (rByte_temp == C_REJ0 || rByte_temp == C_REJ1)
                        {
                            STOP = TRUE;
                        }
                    }
                    break;
                default:
                    break;
                }
            }
        }
        retransmissions_var--;
        

    }
    free(frame);

    if(rByte_temp == C_REJ0 || rByte_temp == C_REJ1)
    {
        llclose(fd);
        return -1;
    } else
    {
        return i; // trocar o return pois está errado
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int llread(unsigned char *packet) {
    unsigned char byte, cField;
    int res = 0;
    State state = START;
    int fd = openConnection(serialPort);

    do {
        if (read(fd, &byte, 1) > 0) {
            res = stateMachinePck(byte, &state, packet, fd);
        }
    } while (state != STOP_STATE);

    return res;
}
////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    State state = START;
    unsigned char rByte;
    (void)signal(SIGALRM, alarmHandler);
    int retransmissions_var = retransmissions;

    while (retransmissions_var > 0 && state != STOP_STATE) {
        alarmCount++;
        alarm(timer);
        alarmEnabled = FALSE;

        while (state != STOP_STATE && alarmEnabled == FALSE) {
            switch (read(showStatistics, &rByte, 1)) {
                case 1:
                    stateMachine(rByte, &state);
                    if (state == STOP_STATE) {
                        printf("Connection established\n");
                    } else
                        return 1;
                    break;
                case -1:
                    return -1;
                    break;
                default:
                    break;
            }
        }
        retransmissions_var--;
    }

    transmitFrame(showStatistics, A_SR, C_UA); // send UA frame
    if(close(showStatistics) == -1) return -1;
    else return 1;
}
