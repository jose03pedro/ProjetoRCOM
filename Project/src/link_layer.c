// Link layer protocol implementation

#include "../include/link_layer.h"

#include "../include/application_layer.h"

int localFrame = 0;

volatile int alarmCount = 0;
volatile int alarmEnabled = FALSE;

int counter = 0;
const char *serialPort;

void alarmHandler(int signal) {
    alarmEnabled = TRUE;
    alarmCount++;

    printf("Alarm %d\n", alarmCount);
}

int retransmissions = 0;
int timer = 0;
volatile int STOP = FALSE;
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

    State state = START;
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
            alarm(timer);
            alarmEnabled = FALSE;
            while (alarmEnabled == FALSE && state != STOP_STATE){
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
                                state = START;
                            break;
                        case C_UA:
                            if (state == A_RCV)
                                state = C_RCV;
                            else
                                state = START;
                            break;
                        case BCC(A_RS, C_UA):
                            if (state == C_RCV)
                                state = BCC1_OK;
                            else
                                state = START;
                            break;
                        default:
                            break;
                    }
                }

            }

            if (state == STOP_STATE) {
                printf("Connection established\n");
            } else {
                printf("State :%d\n", state);
                return -1;
            }
            retransmissions_var--;
        }

        if (state != STOP_STATE) {
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
                            state = START;

                    case A_SR:
                        if (state == FLAG_RCV)
                            state = A_RCV;
                        else if (state == A_RCV)
                            state = C_RCV;
                        else
                            state = START;
                    case C_I0:
                        if (state == C_RCV)
                            state = BCC1_OK;
                        else
                            state = START;
                    default:
                        state = START;
                        break;
                }
                printf("fimss");
                state = STOP_STATE;
                // stateMachineRx(rByte, &state);
            }
        } while (state != STOP_STATE);
        transmitFrame(A_RS, C_UA);  // send UA frame
    } else {
        return -1;
    }
    alarm(0);
    return 1;
}

int frameCounter = 0;
////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    // Creating the frame to send
    printf("LocalFrame: %d\n", localFrame);
    int frameSize = bufSize + 6;
    unsigned char *frame = (unsigned char *)malloc(bufSize + 6);
    frame[0] = FLAG;
    frame[1] = A_SR;
    if (localFrame == 0) {
        frame[2] = C_I0;
    } else if(localFrame == 1){
        frame[2] = C_I1;
    }
    frame[3] = BCC(frame[1], frame[2]);

    int bsize = bufSize;
    unsigned char bcc2 = 0;
    bcc2 = buf[0];
    int var = 0;

    // BCC2
    while (bsize > 0) {
        var++;
        bcc2 ^= buf[var];
        bsize--;
    }

    // Stuffing
    int i = 4;
    int j = 0;
    unsigned char value = 0;
    while (j < bufSize) {
        value = buf[j];
        switch (value) {
            case FLAG:
                frame = realloc(frame, ++frameSize);
                frame[i++] = ESCAPE;
                frame[i++] = ESCAPE_FLAG;
                break;
            case ESCAPE:
                frame = realloc(frame, ++frameSize);
                frame[i++] = ESCAPE;
                frame[i++] = ESCAPE_ESCAPE;
                break;
            default:
                frame[i++] = buf[j++];
                break;
        }
        j++;
    }

    // BCC2 Stuffing
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

    unsigned char rByte_temp = 0;

    int nRetransmissions = 0;
    int accepted = 0;

    // Send frame
    while ((nRetransmissions < retransmissions) && !accepted) {
        alarm(0);
        write(fd, frame, i);  // i = frame size
        printf("Frame sent\n");
        alarm(timer);
        alarmEnabled = FALSE;
        State state = START;

        while (alarmEnabled == FALSE) {
            printf("State_llwrite: %d\n", state);
            unsigned char rByte;
            if(read(fd, &rByte, 1) > 0){
                switch (state) {
                    case START:
                        // printf("rByte: %hhu\n", rByte);
                        // printf("aaa\n");
                        if (rByte == FLAG) {
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if (rByte == A_SR) {
                            state = A_RCV;
                        } else if (rByte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (rByte == C_RR0 || rByte == C_RR1 ||
                            rByte == C_REJ0 || rByte == C_REJ1 || rByte == C_DISC) {
                            state = C_RCV;
                            rByte_temp = rByte;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (rByte == BCC(A_SR, rByte_temp)) {
                            state = BCC1_OK;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC1_OK:
                        if (rByte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                        break;
                    case STOP_STATE:
                        STOP = TRUE;
                        alarm(0);
                        if(rByte_temp == C_RR0 || rByte_temp == C_RR1){
                            accepted = 1;
                            if(rByte_temp == C_RR0) localFrame = 0;
                            else if(rByte_temp == C_RR1) localFrame = 1;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        nRetransmissions++;
    }
    alarmCount = 0;
    free(frame);

    return accepted;  // trocar o return pois está errado
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int llread(unsigned char *packet) {
    unsigned char rByte, cAux, acc, bcc2;
    unsigned int res = 0;
    State state = START;

    while(state != STOP_STATE) {
        if (read(fd, &rByte, 1) > 0) {
            printf("State_llread: %d\n", state);
            switch (state) {
                case START:
                    if (rByte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (rByte == A_SR)
                        state = A_RCV;
                    else if (rByte != FLAG)
                        state = START;
                    break;
                case A_RCV:
                    printf("rByte_arcv %hhu\n", rByte);
                    if (rByte == C_I0 || rByte == C_I1) {
                        if (rByte == C_I0 && localFrame == 0) {
                            localFrame = 1;
                            state = C_RCV;
                            cAux = BCC(rByte, A_SR);
                        } else if (rByte == C_I1 && localFrame == 1) {
                            localFrame = 0;
                            state = C_RCV;
                            cAux = BCC(rByte, A_SR);
                        }
                    } else if (rByte == FLAG)
                        state = FLAG_RCV;
                    else if (rByte == C_DISC) {
                        transmitFrame(A_RS, C_DISC);
                        return 0;
                    } else
                        state = START;
                    break;
                case C_RCV:
                    if (rByte == cAux)
                        state = READING_DATA;
                    else if (rByte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC1_OK:
                    if (rByte == FLAG)
                        state = STOP_STATE;
                    else
                        state = START;
                    break;
/*
case STOP_STATE:
                    if (rByte == FLAG) {
                        printf("sent frame\n");
                        transmitFrame(A_SR, C_UA);  // send UA frame
                        STOP = TRUE;
                        alarm(0);
                    } else {
                        state = START;
                    }
*/
                
                case ESC_FOUND:
                    state = READING_DATA;
                    if (rByte == STUF_ESCAPE)
                        packet[res++] = FLAG;

                    else if (rByte == STUF_FLAG)
                        packet[res++] = ESCAPE;
                    break;
                case READING_DATA:
                    if (rByte == ESCAPE)
                        state = ESC_FOUND;
                    else if (rByte == FLAG) {
                        bcc2 = packet[(res--) - 1];
                        packet[res] = '\0';
                        acc = packet[0];

                        for (unsigned int j = 1; j < res; j++) acc ^= packet[j];

                        if (bcc2 == acc) {
                            state = STOP_STATE;
                            if (localFrame == 0) {
                                printf("localFrame = 0\n");
                                transmitFrame(A_RS, C_RR0);
                            } else if (localFrame == 1) {
                                printf("localFrame = 1\n");
                                transmitFrame(A_RS, C_RR1);
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
                        packet[res++] = rByte;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    return -1;
}
////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    State state = START;
    unsigned char rByte;
    (void)signal(SIGALRM, alarmHandler);
    alarm(0);
    alarmCount = 0;
    alarmEnabled = FALSE;
    if (role == LlTx) {
        while (alarmCount < retransmissions && state != STOP_STATE) {
            if (alarmEnabled == FALSE) {
                transmitFrame(A_SR, C_DISC);  // send DISC frame
                alarm(timer);
                alarmEnabled = TRUE;
                while (STOP == FALSE) {
                    read(fd, &rByte, 1);
                    switch (state) {
                        case START:
                            if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case FLAG_RCV:
                            if (rByte == A_RS) {
                                state = A_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                            break;
                        case A_RCV:
                            if (rByte == C_DISC) {
                                state = C_RCV;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                        case C_RCV:
                            if (rByte == BCC(A_RS, C_DISC)) {
                                state = BCC1_OK;
                            } else if (rByte == FLAG) {
                                state = FLAG_RCV;
                            } else {
                                state = START;
                            }
                        case BCC1_OK:
                            if (rByte == FLAG) {
                                state = STOP_STATE;
                            } else {
                                state = START;
                            }
                            // case STOP_STATE:
                            //     if (rByte == FLAG) {
                            //         transmitFrame(A_SR, C_UA);  // send UA
                            //         frame STOP = TRUE; alarm(0);
                            //     } else {
                            //         state = START;
                            //     }
                            break;
                        default:
                            break;
                    }
                }
            }
            alarmCount++;
        }
        if (state != STOP_STATE) return -1;
        transmitFrame(A_SR, C_UA);
        close(fd);
    } else if (role == LlRx) {
        unsigned char rByte2;
        while (STOP == FALSE) {
            read(fd, &rByte2, 1);
            switch (state) {
                case START:
                    if (rByte2 == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case FLAG_RCV:
                    if (rByte2 == A_RS) {
                        state = A_RCV;
                    } else if (rByte2 == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (rByte2 == C_DISC) {
                        state = C_RCV;
                    } else if (rByte2 == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                case C_RCV:
                    if (rByte2 == BCC(A_RS, C_DISC)) {
                        state = BCC1_OK;
                    } else if (rByte2 == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                case BCC1_OK:
                    if (rByte2 == FLAG) {
                        state = STOP_STATE;
                    } else {
                        state = START;
                    }
                case STOP_STATE:
                    if (rByte2 == FLAG) {
                        transmitFrame(A_SR, C_DISC);  // send DISC frame
                        STOP = TRUE;
                    } else {
                        state = START;
                    }
                    break;
                default:
                    break;
            }
            // }
            // alarmCount = 0;
            // STOP = FALSE;
            // alarmEnabled = FALSE;
            // State state = START;

            // while (STOP == FALSE) {
            //     read(fd, &rByte, 1);
            //     switch (state) {
            //         case START:
            //             if (rByte == FLAG) {
            //                 state = FLAG_RCV;
            //             } else {
            //                 state = START;
            //             }
            //             break;
            //         case FLAG_RCV:
            //             if (rByte == A_RS) {
            //                 state = A_RCV;
            //             } else if (rByte == FLAG) {
            //                 state = FLAG_RCV;
            //             } else {
            //                 state = START;
            //             }
            //             break;
            //         case A_RCV:
            //             if (rByte == C_UA) {
            //                 state = C_RCV;
            //             } else if (rByte == FLAG) {
            //                 state = FLAG_RCV;
            //             } else {
            //                 state = START;
            //             }
            //         case C_RCV:
            //             if (rByte == BCC(A_RS, C_UA)) {
            //                 state = BCC1_OK;
            //             } else if (rByte == FLAG) {
            //                 state = FLAG_RCV;
            //             } else {
            //                 state = START;
            //             }
            //         case BCC1_OK:
            //             if (rByte == FLAG) {
            //                 state = STOP_STATE;
            //             } else {
            //                 state = START;
            //             }
            //         case STOP_STATE:
            //             if (rByte == FLAG) {
            //                 transmitFrame(A_SR, C_UA);  // send UA frame
            //                 STOP = TRUE;
            //             } else {
            //                 state = START;
            //             }
            //             break;
            //         default:
            //             break;
        }
        transmitFrame(A_SR, C_DISC);
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
            if (byte == BCC(A_RS, C_DISC))
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
    printf("ohyeeee %hhu\n", byte);
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            printf("2222222\n");
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

            if (byte == BCC(A_RS, C_UA))
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
    printf("State :%d\n", *state);
    printf("Byte :%hhu\n", byte);
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            printf("bytee %hhu\n", byte);
            if (byte == A_SR) {
                *state = A_RCV;
            } else if (byte != FLAG)
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
            if (byte == BCC(A_SR, C_SET))
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
    State curr_state = START;
    unsigned char byte_read;
    unsigned char controlField = 0;

    while (alarmEnabled == FALSE) {
        unsigned char bytes_read = read(fd, &byte_read, 1);

        if (bytes_read > 0) {
            switch (curr_state) {
                case START: {
                    if (byte_read == FLAG) curr_state = FLAG_RCV;
                    break;
                }

                case FLAG_RCV: {
                    if (byte_read == A_SR)
                        curr_state = A_RCV;
                    else if (byte_read != FLAG)
                        curr_state = START;
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
                        curr_state = START;
                    break;
                }

                case C_RCV: {
                    if (byte_read == (A_SR ^ controlField))
                        curr_state = BCC1_OK;
                    else if (byte_read == FLAG)
                        curr_state = FLAG_RCV;
                    else
                        curr_state = START;
                    break;
                }

                case BCC1_OK: {
                    if (byte_read == FLAG) {
                        return controlField;
                    } else
                        curr_state = START;
                    break;
                }

                default:
                    break;
            }
        }
    }

    return 0;
}