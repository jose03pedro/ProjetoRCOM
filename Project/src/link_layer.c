// Link layer protocol implementation

#include "../include/link_layer.h"
#include "../include/application_layer.h"

unsigned char txFrame = 0;
unsigned char rxFrame = 1;

int alarmCount = 0;
int alarmEnabled = FALSE;
const char *serialPort;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
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

    // fd = openConnection(connectionParameters.serialPort);
    //  printf("inside llopen\n");

    State state = START;
    // printf("fd_llopen: %d\n", fd);
    if (fd < 0) {
        return -1;
    }
    printf("fd_llopen2: %d\n", fd);

    role = connectionParameters.role;

    unsigned char rByte;
    timer = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;
    serialPort = connectionParameters.serialPort;

    int retransmissions_var = retransmissions;
    // printf(" before role == LlTx\n");
    if (role == LlTx) {
        printf("inside role\n");
        (void)signal(SIGALRM, alarmHandler);

        do {
            // printf("inside do-while\n");
            transmitFrame(fd, A_SR, C_SET);  // send SET frame
            alarm(timer);
            alarmEnabled = FALSE;

            do {
                switch (read(fd, &rByte, 1)) {
                    case 1:
                        // printf(" before statemachine\n");
                        stateMachineTx(rByte, &state);
                        // printf(" after statemachine\n");
                        if (state == STOP_STATE) {
                            printf("Connection established\n");
                        }
                        break;
                    case -1:
                        return -1;
                        break;
                    default:
                        // printf("byte: %hhu\n", rByte);
                        // printf("fd :%d\n", fd);
                        break;
                }
            } while (alarmEnabled == FALSE && state != STOP_STATE);

            retransmissions_var--;
        } while (retransmissions_var != 0 && state != STOP_STATE);

        if (state != STOP_STATE) return -1;
    } else if (role == LlRx) {
        do {
            // printf("rbyte: %hhu\n", rByte);
            // size_t test = read(fd, &rByte, 1);
            // printf("read output: %d\n", test);
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
                    // printf("rbyte: %d\n", rByte);
                    break;
            }
        } while (state != STOP_STATE);

        transmitFrame(fd, A_RS, C_UA);  // send UA frame
    } else {
        return -1;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
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
                frame[i] = ESCAPE;
                frame[i + 1] = ESCAPE_FLAG;
                i += 2;
                break;
            case ESCAPE:
                frame[i] = ESCAPE;
                frame[i + 1] = ESCAPE_ESCAPE;
                i += 2;
                break;
            default:
                frame[i] = buf[j];
                i++;
                break;
        }
        j++;
    }

    // BCC2 Stuffing
    switch (bcc2) {
        case FLAG:
            frame[i] = ESCAPE;
            frame[i + 1] = ESCAPE_FLAG;
            i += 2;
            break;
        case ESCAPE:
            frame[i] = ESCAPE;
            frame[i + 1] = ESCAPE_ESCAPE;
            i += 2;
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
    while (retransmissions_var > 0) {
        write(fd, frame, i);  // i = frame size

        alarm(timer);
        alarmEnabled = TRUE;

        while (STOP == FALSE && alarmEnabled == TRUE) {
            unsigned char rByte;
            read(fd, &rByte, 1);
            while (state != STOP_STATE) {
                switch (state) {
                    case START:
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
                            rByte == C_REJ0 || rByte == C_REJ1) {
                            state = C_RCV;
                            rByte_temp = rByte;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;

                    case C_RCV:
                        if (rByte == BCC(A_SR, C_I0) ||
                            rByte == BCC(A_SR, C_I1)) {
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
                        if (rByte == FLAG) {
                            if (rByte_temp == C_RR0 || rByte_temp == C_RR1) {
                                if (rByte_temp == C_RR0) {
                                    txFrame = 1;
                                    rxFrame = 0;
                                } else {
                                    txFrame = 0;
                                    rxFrame = 1;
                                }
                                STOP = TRUE;
                                alarm(0);
                                retransmissions_var = 0;
                            } else if (rByte_temp == C_REJ0 ||
                                       rByte_temp == C_REJ1) {
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

    if (rByte_temp == C_REJ0 || rByte_temp == C_REJ1) {
        // llclose(fd);
        return -1;
    } else {
        return i;  // trocar o return pois está errado
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int llread(unsigned char *packet) {
    unsigned char byte;
    int res = 0;
    State state = START;

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
    alarmCount = 0;
    alarmEnabled = FALSE;

    if (role == LlTx) {
        while (alarmCount < retransmissions && state != STOP_STATE) {
            if (alarmEnabled == FALSE) {
                transmitFrame(fd, A_SR, C_DISC);  // send DISC frame
                alarm(timer);
                alarmEnabled = TRUE;
                while (STOP == FALSE) {
                    read(fd, &rByte, 1);
                    if (state == START) {
                        if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                    } else if (state == FLAG_RCV) {
                        if (rByte == A_RS) {
                            state = A_RCV;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                    } else if (state == A_RCV) {
                        if (rByte == C_DISC) {
                            state = C_RCV;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                    } else if (state == C_RCV) {
                        if (rByte == BCC(A_RS, C_DISC)) {
                            state = BCC1_OK;
                        } else if (rByte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                    } else if (state == BCC1_OK) {
                        if (rByte == FLAG) {
                            state = STOP_STATE;
                        } else {
                            state = START;
                        }
                    } else if (state == STOP_STATE) {
                        if (rByte == FLAG) {
                            transmitFrame(fd, A_SR, C_UA);  // send UA frame
                            STOP = TRUE;
                            alarm(0);
                        } else {
                            state = START;
                        }
                    }
                }
            }
        }
    } else if (role == LlRx) {
        while (STOP == FALSE) {
            read(fd, &rByte, 1);
            if (state == START) {
                if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == FLAG_RCV) {
                if (rByte == A_RS) {
                    state = A_RCV;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == A_RCV) {
                if (rByte == C_DISC) {
                    state = C_RCV;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == C_RCV) {
                if (rByte == BCC(A_RS, C_DISC)) {
                    state = BCC1_OK;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == BCC1_OK) {
                if (rByte == FLAG) {
                    state = STOP_STATE;
                } else {
                    state = START;
                }
            } else if (state == STOP_STATE) {
                if (rByte == FLAG) {
                    transmitFrame(fd, A_SR, C_DISC);  // send DISC frame
                    STOP = TRUE;
                } else {
                    state = START;
                }
            }
        }
        alarmCount = 0;
        STOP = FALSE;
        alarmEnabled = FALSE;
        State state = START;

        while (STOP == FALSE) {
            read(fd, &rByte, 1);
            if (state == START) {
                if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == FLAG_RCV) {
                if (rByte == A_RS) {
                    state = A_RCV;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == A_RCV) {
                if (rByte == C_UA) {
                    state = C_RCV;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == C_RCV) {
                if (rByte == BCC(A_RS, C_UA)) {
                    state = BCC1_OK;
                } else if (rByte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
            } else if (state == BCC1_OK) {
                if (rByte == FLAG) {
                    state = STOP_STATE;
                } else {
                    state = START;
                }
            } else if (state == STOP_STATE) {
                if (rByte == FLAG) {
                    transmitFrame(fd, A_SR, C_UA);  // send UA frame
                    STOP = TRUE;
                } else {
                    state = START;
                }
            }
        }
    }
    /*
    if(tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }
    */
    if (close(fd) == -1)
        return -1;
    else
        return 1;
}


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
        case ESC_FOUND:
            *state = READING_DATA;
            if (byte == ESCAPE || byte == FLAG)
                packet[i++] = byte;
            else {
                packet[i++] = ESCAPE;
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
                    if (rxFrame == 0)
                        transmitFrame(fd, A_RS, C_REJ0);
                    else if (rxFrame == 1)
                        transmitFrame(fd, A_RS, C_REJ1);
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
    printf("State :%d\n", *state);
    printf("Byte :%hhu\n", byte);
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
