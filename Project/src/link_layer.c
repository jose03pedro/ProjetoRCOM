// Link layer protocol implementation

#include "../include/link_layer.h"

#include "../include/application_layer.h"

unsigned char localFrame = 0;

int alarmCount = 0;
int alarmEnabled = FALSE;

int counter = 0;
const char *serialPort;

void alarmHandler(int signal) {
    alarmEnabled = TRUE;
    alarmCount++;

    printf("Alarm %d\n", alarmCount);
}

int retransmissions = 0;
int timer = 0;
volatile int ALARM_STOP = FALSE;
LinkLayerRole role;

int fd;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters) {
    fd = openConnection(connectionParameters.serialPort);  // Abre a conexÃ£o
    //  serial
    if (fd < 0) {
        printf("ret1\n");
        return -1;
    }

    State state = START_STATE;
    role = connectionParameters.role;
    timer = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;
    serialPort = connectionParameters.serialPort;

    int retransmissions_var = retransmissions;
    unsigned char rByte;

    if (role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);

        while (retransmissions_var > 0 && state != STOP_STATE) {
            // printf("inside do-while\n");
            transmitFrame(A_SR, C_SET);  // send SET frame
            alarm(connectionParameters.timeout);
            alarmEnabled = FALSE;
            do {
                printf("before read? %d\n", fd);

                if (read(fd, &rByte, 1) > 0) {
                    printf("State_llopen = %d\n", state);
                    printf("Bytellopen: %hhu\n", rByte);
                    // stateMachineTx(rByte, &state);
                    switch (rByte) {
                        case FLAG:
                            if (state != BCC1_OK)
                                state = FLAG_RCV;
                            else
                                printf("stop\n");
                            state = STOP_STATE;
                            break;
                        case C_REJ0:
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

            if (state == STOP_STATE) {
                printf("Connection established\n");
            } else {
                printf("State :%d\n", state);
                printf("ret2\n");
                return -1;
            }
            retransmissions_var--;
        }

        if (state != STOP_STATE) {
            printf("ret3\n");
            return -1;
        }
    } else if (role == LlRx) {
        printf("Inside receiver role \n");
        do {
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
                    case C_I0:
                        if (state == C_RCV)
                            state = BCC1_OK;
                        else
                            state = START_STATE;
                    default:
                        state = START_STATE;
                        break;
                }
                printf("fimss");
                state = STOP_STATE;
                // stateMachineRx(rByte, &state);
            }
        } while (state != STOP_STATE);
        transmitFrame(A_RS, C_UA);  // send UA frame
    } else {
        printf("ret4\n");
        return -1;
    }
    alarm(0);
    printf("ret5\n");
    return 1;
}

int frameCounter = 0;
////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize) {
    printf("LocalFrame: %d\n", localFrame);
    int frameSize = bufSize + 6;
    unsigned char *frame = (unsigned char *)malloc(bufSize + 6);
    frame[0] = FLAG;
    frame[1] = A_SR;
    if (localFrame == 0) {
        frame[2] = C_I0;
    } else if (localFrame == 1) {
        frame[2] = C_I1;
    }
    frame[3] = BCC(frame[1], frame[2]);

    int bsize = bufSize;
    // unsigned char bcc2 = 0;
    // bcc2 = buf[0];
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
            frame[i++] = ESCAPE;
            frame[i++] = buf[j] ^ 0x20;
            j++;
        }

        else if (j < bufSize)
            frame[i++] = buf[j++];
    }

    // bcc2 stuffing
    switch (bcc2) {
        case FLAG:
            frame = realloc(frame, ++frameSize);
            frame[i++] = ESCAPE;
            frame[i++] = ESCAPE_FLAG;
            frame[i++] = FLAG;
            break;
        case ESCAPE:
            frame = realloc(frame, ++frameSize);
            frame[i++] = ESCAPE;
            frame[i++] = ESCAPE_ESCAPE;
            frame[i++] = FLAG;
            break;
        default:
            frame[i++] = bcc2;
            frame[i++] = FLAG;
            break;
    }

    int retransmissions_var = retransmissions;
    int accepted = 0;

    while (!accepted && (retransmissions_var > 0)) {
        printf("Writing...\n");
        alarm(0);
        write(fd, frame, i);
        alarm(timer);
        alarmEnabled = FALSE;
        State state = START_STATE;

        unsigned char rByte_temp = 0, ctrlF = 0;
        printf("Retranmissions %d\n", retransmissions_var);
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
                    case STOP_STATE:
                        ALARM_STOP = TRUE;
                        alarm(0);
                        if (rByte_temp == C_RR0 || rByte_temp == C_RR1) {
                            accepted = 1;
                            if (rByte_temp == C_RR0)
                                localFrame = 0;
                            else if (rByte_temp == C_RR1)
                                localFrame = 1;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        retransmissions_var--;
    }
    alarmCount = 0;
    free(frame);
    if (accepted)
        return 1;
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
    unsigned char eByte;
    int retranmissions_var = retransmissions;
    int STOP = FALSE;
    printf("retranmissions_var : %d\n", retranmissions_var);
    switch (role) {
        case LlTx:
            (void)signal(SIGALRM, alarmHandler);
            alarm(0);
            // int nTry = 0;

            while (retranmissions_var > 0) {
                // send disconnect frame
                if (transmitFrame(A_SR, C_DISC) < 0) {
                    perror("Error writing disconnect frame\n");
                    return -1;
                }

                alarm(timer);
                alarmEnabled = FALSE;

                while (alarmEnabled == FALSE && STOP == FALSE) {
                    if (read(fd, &eByte, 1) > 0) {
                        switch (state) {
                            case START_STATE:
                                if (eByte == FLAG) {
                                    state = FLAG_RCV;
                                }
                                break;
                            case FLAG_RCV:
                                if (eByte == A_RS) {
                                    state = A_RCV;
                                } else if (eByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case A_RCV:
                                if (eByte == C_DISC) {
                                    state = C_RCV;
                                } else if (eByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case C_RCV:
                                if (eByte == BCC(A_RS, C_DISC)) {
                                    state = BCC1_OK;
                                } else if (eByte == FLAG) {
                                    state = FLAG_RCV;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case BCC1_OK:
                                if (eByte == FLAG) {
                                    state = STOP_STATE;
                                } else {
                                    state = START_STATE;
                                }
                                break;
                            case STOP_STATE:
                                transmitFrame(A_SR, C_UA);
                                alarm(0);
                                close(fd);
                                STOP = TRUE;
                                break;
                            default:
                                break;
                        }
                    }
                }
                retranmissions_var--;
            }

            if (state != STOP_STATE) {
                perror("Error receiving DISC frame\n");
                return -1;
            }

            break;

        case LlRx:
            while (STOP == FALSE) {
                if (read(fd, &eByte, 1) > 0) {
                    switch (state) {
                        case START_STATE:
                            if (eByte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (eByte == A_RS) {
                                state = A_RCV;
                            } else if (eByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case A_RCV:
                            if (eByte == C_DISC) {
                                state = C_RCV;
                            } else if (eByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case C_RCV:
                            if (eByte == BCC(A_RS, C_DISC)) {
                                state = BCC1_OK;
                            } else if (eByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case BCC1_OK:
                            if (eByte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START_STATE;
                            }
                            break;
                        case STOP_STATE:
                            transmitFrame(A_RS, C_DISC);
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

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
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

unsigned char readFrame() {
    State curr_state = START_STATE;
    unsigned char byte_read;
    unsigned char controlField = 0;

    while (alarmEnabled == FALSE) {
        unsigned char bytes_read = read(fd, &byte_read, 1);

        if (bytes_read > 0) {
            switch (curr_state) {
                case START_STATE: {
                    if (byte_read == FLAG) curr_state = FLAG_RCV;
                    break;
                }

                case FLAG_RCV: {
                    if (byte_read == A_SR)
                        curr_state = A_RCV;
                    else if (byte_read != FLAG)
                        curr_state = START_STATE;
                    break;
                }

                case A_RCV: {
                    if (byte_read == FLAG)
                        curr_state = FLAG_RCV;
                    else if (byte_read == C_RR0 || byte_read == C_RR1 ||
                             byte_read == C_REJ0 || byte_read == C_REJ1 ||
                             byte_read == C_DISC) {
                        curr_state = C_RCV;
                        controlField = byte_read;
                    } else
                        curr_state = START_STATE;
                    break;
                }

                case C_RCV: {
                    if (byte_read == (A_SR ^ controlField))
                        curr_state = BCC1_OK;
                    else if (byte_read == FLAG)
                        curr_state = FLAG_RCV;
                    else
                        curr_state = START_STATE;
                    break;
                }

                case BCC1_OK: {
                    if (byte_read == FLAG) {
                        return controlField;
                    } else
                        curr_state = START_STATE;
                    break;
                }

                default:
                    break;
            }
        }
    }

    return 0;
}