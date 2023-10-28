// Link layer protocol implementation

#include "../include/link_layer.h"

#include "../include/application_layer.h"

unsigned char localFrame = 0;

int alarmCount = 0;
int alarmEnabled = FALSE;

int counter = 0;

void alarmHandler(int signal) {
    alarmEnabled = TRUE;
    alarmCount++;

    printf("Alarm %d\n", alarmCount);
}

int retransmissions = 0;
int timer = 0;
int baudRate = 0;
volatile int ALARM_STOP = FALSE;
const char *serialPort;
LinkLayerRole role;

struct termios oldtio, newtio;

int fd;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters) {
    State state = START_STATE;
    setGlobalVars(connectionParameters);
    fd = openConnection(connectionParameters.serialPort);  // Abre a conexão
    if (fd < 0) {
        perror("Error opening connection\n");
        exit(-1);
    }

    int retransmissions_var = retransmissions;

    if (role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);

        while (retransmissions_var > 0 && state != STOP_STATE) {
            transmitFrame(A_SR, C_SET);  // send SET frame (sent 1st)
            alarm(connectionParameters.timeout);
            alarmEnabled = FALSE;
            do {
                unsigned char rByte;
                if (read(fd, &rByte, 1) > 0) {
                    switch (rByte) {
                        case FLAG:
                            if (state != BCC1_OK)
                                state = FLAG_RCV;
                            else
                                state = STOP_STATE;
                            break;
                        case A_RS:
                            if (state == FLAG_RCV)
                                state = A_RCV;
                            else
                                state = START_STATE;
                            break;
                        case C_UA:
                            if (state == A_RCV)
                                state = C_RCV;
                            else
                                state = START_STATE;
                            break;
                        case BCC(A_RS, C_UA):
                            if (state == C_RCV)
                                state = BCC1_OK;
                            else
                                state = START_STATE;
                            break;
                        default:
                            break;
                    }
                }

            } while (alarmEnabled == FALSE && state != STOP_STATE);

            retransmissions_var--;
        }

        if (state != STOP_STATE) {
            perror("Error establishing connection\n");
            return -1;
        }

    } else if (role == LlRx) {
        do {
            unsigned char rByte;
            if (read(fd, &rByte, 1) > 0) {
                switch (rByte) {
                    case FLAG:
                        if (state == FLAG_RCV)
                            state = A_RCV;
                        else if (state == A_RCV)
                            state = C_RCV;
                        else
                            state = START_STATE;

                    case A_SR:
                        if (state == FLAG_RCV)
                            state = A_RCV;
                        else if (state == A_RCV)
                            state = C_RCV;
                        else
                            state = START_STATE;
                    case BCC(A_SR, C_SET):
                        if (state == C_RCV)
                            state = BCC1_OK;
                        else
                            state = START_STATE;
                    default:
                        state = START_STATE;
                        break;
                }
                state = STOP_STATE;
            }
        } while (state != STOP_STATE);
        transmitFrame(A_RS, C_UA);  // send UA frame (sent 2nd)
    } else {
        perror("Error: Invalid role\n");
        return -1;
    }
    alarm(0);
    printf("Connection established\n");
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize) {
    // printf("LocalFrame: %d\n", localFrame);
    int frameSize = bufSize;
    unsigned char *frame = (unsigned char *)malloc(bufSize + 6);
    frame[0] = FLAG;
    frame[1] = A_SR;
    frame[2] = (localFrame == 0) ? C_I0 : C_I1;
    frame[3] = BCC(frame[1], frame[2]);

    int bsize = bufSize;
    int var = 0;

    memcpy(frame + 4, buf, bsize);

    unsigned char bcc2 = buf[0];

    // calculates the bcc2
    while (bsize > 0) {
        var++;
        bcc2 ^= buf[var];
        bsize--;
    }

    // stuffing
    int i = 4;
    int j = 0;
    unsigned char value = 0;
    while (j < bufSize) {
        value = buf[j];

        if (value == FLAG || value == ESCAPE) {
            frame = realloc(frame, ++frameSize);
            frame[i] = ESCAPE;
            frame[i+1] = buf[j] ^ STUFFING;
            i += 2;
            j++;
        }

        else if (j < bufSize){
            frame[i] = buf[j];
            i++;
            j++;
        }
            
    }

    // bcc2 stuffing
    switch (bcc2) {
        case FLAG:
            frame = realloc(frame, ++frameSize);
            frame[i] = ESCAPE;
            frame[i+1] = ESCAPE_FLAG;
            frame[i+2] = FLAG;
            i += 3;
            break;
        case ESCAPE:
            frame = realloc(frame, ++frameSize);
            frame[i] = ESCAPE;
            frame[i+1] = ESCAPE_ESCAPE;
            frame[i+2] = FLAG;
            i += 3;
            break;
        default:
            frame[i] = bcc2;
            frame[i+1] = FLAG;
            i += 2;
            break;
    }

    int retransmissions_var = retransmissions;
    int accepted = 0;
    unsigned char ctrlF = 0;

    while (!accepted && (retransmissions_var > 0)) {
        printf("Writing...\n");
        alarm(0);
        write(fd, frame, i);
        alarm(timer);
        alarmEnabled = FALSE;
        State state = START_STATE;

        unsigned char rByte_temp = 0;
        ctrlF = 0;
        while (alarmEnabled == FALSE) {
            // unsigned char rByte;
            if (read(fd, &rByte_temp, 1) > 0) {
                switch (state) {
                    case START_STATE:
                        if (rByte_temp == FLAG) {
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if (rByte_temp == A_RS) {
                            state = A_RCV;
                        } else if (rByte_temp != FLAG) {
                            state = START_STATE;
                        }
                        break;
                    case A_RCV:
                        if (rByte_temp == C_RR0 || rByte_temp == C_RR1 ||
                            rByte_temp == C_REJ0 || rByte_temp == C_REJ1 ||
                            rByte_temp == C_DISC) {
                            state = C_RCV;
                            ctrlF = rByte_temp;
                        } else if (rByte_temp == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START_STATE;
                        }
                        break;
                    case C_RCV:
                        if (rByte_temp == BCC(A_RS, ctrlF)) {
                            state = BCC1_OK;
                        } else if (rByte_temp == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START_STATE;
                        }
                        break;
                    case BCC1_OK:
                        if (rByte_temp == FLAG) {
                            return ctrlF;
                        } else {
                            state = START_STATE;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        if (ctrlF == C_RR0 || ctrlF == C_RR1) {
            accepted = 1;
            if (ctrlF == C_RR0)
                localFrame = 0;
            else if (ctrlF == C_RR1)
                localFrame = 1;
        }
        retransmissions_var--;
    }
    alarmCount = 0;
    free(frame);
    if (accepted){
        llclose(1);
        return ctrlF;
    }
    else {
        llclose(1);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int llread(unsigned char *packet) {
    unsigned char r_byte, cAux, bcc2, acc;
    unsigned int res = 0;
    State r_state = START_STATE;

    do {
        // printf("LocalFrame: %d\n", localFrame);
        if (read(fd, &r_byte, 1) > 0) {
            // printf("State_llread: %d\n", r_state);
            switch (r_state) {
                case START_STATE:
                    if (r_byte == FLAG) r_state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (r_byte == A_SR)
                        r_state = A_RCV;
                    else if (r_byte != FLAG)
                        r_state = START_STATE;
                    break;
                case A_RCV:
                    printf("byte_arcv %hhu\n", r_byte);
                    if (r_byte == C_I0 || r_byte == C_I1) {
                        if (r_byte == C_I0 && localFrame == 0) {
                            localFrame++;
                            r_state = C_RCV;
                            cAux = BCC(r_byte, A_SR);
                        } else if (r_byte == C_I1 && localFrame == 1) {
                            localFrame--;
                            r_state = C_RCV;
                            cAux = BCC(r_byte, A_SR);
                        }
                    } else if (r_byte == FLAG)
                        r_state = FLAG_RCV;
                    else if (r_byte == C_DISC) {
                        transmitFrame(A_RS, C_DISC);
                        return 0;
                    } else
                        r_state = START_STATE;
                    break;
                case C_RCV:
                    if (r_byte == cAux)
                        r_state = READING_DATA;
                    else if (r_byte == FLAG)
                        r_state = FLAG_RCV;
                    else
                        r_state = START_STATE;
                    break;
                case BCC1_OK:
                    if (r_byte == FLAG)
                        r_state = STOP_STATE;
                    else
                        r_state = START_STATE;
                    break;

                case STOP_STATE:
                    if (r_byte == FLAG) {
                        printf("sent frame\n");
                        transmitFrame(A_SR, C_UA);  // send UA frame
                        ALARM_STOP = TRUE;
                        alarm(0);
                    } else {
                        r_state = START_STATE;
                    }
                case ESC_FOUND:
                    r_state = READING_DATA;
                    if (r_byte == STUF_ESCAPE)
                        packet[res++] = ESCAPE;

                    else if (r_byte == STUF_FLAG)
                        packet[res++] = FLAG;
                    else
                        printf("stuffing error\n");
                    printf("Iterator: %d\n", res);
                    break;
                case READING_DATA:
                    if (r_byte == ESCAPE)
                        r_state = ESC_FOUND;
                    else if (r_byte == FLAG) {
                        bcc2 = packet[(res--) - 1];
                        printf("res =  %d\n", res);
                        packet[res] = '\0';
                        acc = packet[0];
                        for (int j = 1; j < res; j++) acc ^= packet[j];
                        printf("bcc2 : %hhu\n", bcc2);
                        printf("acc : %hhu\n", acc);
                        if (bcc2 == acc) {
                            r_state = STOP_STATE;
                            if (localFrame == 0) {
                                printf("localFrame = 0\n");
                                transmitFrame(A_RS, C_RR0);
                                localFrame = 1;
                            } else if (localFrame == 1) {
                                printf("localFrame = 1\n");
                                transmitFrame(A_RS, C_RR1);
                                localFrame = 0;
                            }
                            return res;
                        } else {
                            printf("Error: retransmition\n");
                            if (localFrame == 0) {
                                printf("localFrame = 0\n");
                                transmitFrame(A_RS, C_REJ0);
                            } else if (localFrame == 1) {
                                transmitFrame(A_RS, C_REJ1);
                                printf("localFrame = 1\n");
                            }
                            return -1;
                        };

                    } else {
                        packet[res++] = r_byte;
                    }
                    break;
                default:
                    break;
            }
        }
    } while (r_state != STOP_STATE);

    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    printf("Inside llclose\n");
    State state = START_STATE;
    int retranmissions_var = retransmissions;
    int STOP = FALSE;
    printf("retranmissions_var : %d\n", retranmissions_var);
    switch (role) {
        case LlTx:
            (void)signal(SIGALRM, alarmHandler);
            alarm(0);
            // int nTry = 0;
            alarmEnabled = FALSE;
            STOP = FALSE;
            while (retranmissions_var > 0) {
                // send disconnect frame
                if (transmitFrame(A_SR, C_DISC) < 0) {
                    perror("Error writing disconnect frame\n");
                    return -1;
                }

                alarm(timer);

                while (alarmEnabled == FALSE && STOP == FALSE) {
                    unsigned char rByte;
                    printf("inside while\n");
                    if (read(fd, &rByte, 1) > 0) {
                        printf("rByte :%hhu\n", rByte);
                        printf("State :%d\n", state);
                        switch (state) {
                            case START_STATE:
                                if (rByte == FLAG) {
                                    state = FLAG_RCV;
                                }
                                break;
                            case FLAG_RCV:
                                if (rByte == A_RS) {
                                    state = A_RCV;
                                } else if (rByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case A_RCV:
                                if (rByte == C_DISC) {
                                    state = C_RCV;
                                } else if (rByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case C_RCV:
                                if (rByte == BCC(A_RS, C_DISC)) {
                                    state = BCC1_OK;
                                } else if (rByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case BCC1_OK:
                                if (rByte == FLAG) {
                                    printf("Flag\n");
                                    state = STOP_STATE;
                                    alarmEnabled = TRUE;
                                    STOP = TRUE;
                                } else {
                                    state = START_STATE;
                                }
                                break;

                            default:
                                break;
                        }
                    } else {
                        printf("Sending UA frame123123123\n");
                        transmitFrame(A_SR, C_UA);
                        alarm(0);
                        alarmEnabled = TRUE;
                        STOP = TRUE;
                    }
                }
                retranmissions_var--;
            }

            break;

        case LlRx:
            while (STOP == FALSE) {
                unsigned char rByte;
                if (read(fd, &rByte, 1) > 0) {
                    switch (state) {
                        case START_STATE:
                            if (rByte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (rByte == A_SR) {
                                state = A_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case A_RCV:
                            if (rByte == C_DISC) {
                                state = C_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case C_RCV:
                            if (rByte == BCC(A_SR, C_DISC)) {
                                state = BCC1_OK;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case BCC1_OK:
                            if (rByte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case STOP_STATE:
                            transmitFrame(A_RS, C_DISC);
                            printf("Sending DISC frame\n");
                            STOP = TRUE;
                            break;
                        default:
                            break;
                    }
                }
            }
            STOP = FALSE;
            while (STOP == FALSE) {
                unsigned char rByte;
                if (read(fd, &rByte, 1) > 0) {
                    switch (state) {
                        case START_STATE:
                            if (rByte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (rByte == A_SR) {
                                state = A_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case A_RCV:
                            if (rByte == C_UA) {
                                state = C_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case C_RCV:
                            if (rByte == BCC(A_SR, C_UA)) {
                                state = BCC1_OK;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case BCC1_OK:
                            if (rByte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case STOP_STATE:
                            printf("Sending asdasd frame\n");
                            STOP = TRUE;
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        default:
            break;
    }
    close(fd);
    return 1;
}
/*
if(tcsetattr(fd, TCSANOW, &oldtio) == -1){
    perror("tcsetattr");
    exit(-1);
}
*/

////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////

int transmitFrame(unsigned char A, unsigned char C) {
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
        case START_STATE:
            if (byte == FLAG) *state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == A_RS)
                *state = A_RCV;
            else if (byte != FLAG)
                *state = START_STATE;
            break;

        case A_RCV:
            if (byte == C_DISC)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;

        case C_RCV:
            if (byte == BCC(A_RS, C_DISC))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;

        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START_STATE;
            break;

        default:
            break;
    }
}

void stateMachineTx(unsigned char byte, State *state) {
    printf("ohyeeee %hhu\n", byte);
    switch (*state) {
        case START_STATE:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            printf("2222222\n");
            if (byte == A_RS)
                *state = A_RCV;
            else if (byte != FLAG)
                *state = START_STATE;
            break;
        case A_RCV:
            if (byte == C_UA)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;
        case C_RCV:

            if (byte == BCC(A_RS, C_UA))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;
        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START_STATE;
            break;
        default:
            break;
    }
}

void stateMachineRx(unsigned char byte, State *state) {
    printf("State :%d\n", *state);
    printf("Byte :%hhu\n", byte);
    switch (*state) {
        case START_STATE:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            printf("bytee %hhu\n", byte);
            if (byte == A_SR) {
                *state = A_RCV;
            } else if (byte != FLAG)
                *state = START_STATE;
            break;
        case A_RCV:
            if (byte == C_SET)
                *state = C_RCV;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;
        case C_RCV:
            if (byte == BCC(A_SR, C_SET))
                *state = BCC1_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START_STATE;
            break;
        case BCC1_OK:
            if (byte == FLAG)
                *state = STOP_STATE;
            else
                *state = START_STATE;
            break;
        default:
            break;
    }
}

int openConnection(const char *serialPort) {

    fd = open(serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPort);
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 10;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    return fd;
}

void setGlobalVars(LinkLayer connectionParameters) {
    role = connectionParameters.role;
    timer = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;
    serialPort = connectionParameters.serialPort;
    baudRate = connectionParameters.baudRate;
}