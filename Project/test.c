// Link layer protocol implementation

#include "link_layer.h"
// MISC
#define _POSIX_SOURCE 1  // POSIX compliant source
int alarmFlag = FALSE;
int alarmCounter = 0;
int delay = 0;
int retryCount = 0;
int cpRetransmissions = 0;
int tramaT = 0;
int tramaR = 0;
int fd;

struct termios oldtio;
struct termios newtio;

LinkLayerRole role_;

void alarmHandler(int signal) {
    alarmFlag = TRUE;
    alarmCounter++;
}

/////////////////////////////////////////////////////////////////////////
//                            LLOPEN                                   //
/////////////////////////////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters) {
    State state = START;
    int fd = makeConnection(connectionParameters.serialPort);
    if (fd < 0) return -1;
    role_ = connectionParameters.role;
    unsigned char byte;
    delay = connectionParameters.timeout;
    retryCount = connectionParameters.nRetransmissions;
    cpRetransmissions = connectionParameters.nRetransmissions;
    switch (connectionParameters.role) {
        case LlTx: {
            (void)signal(SIGALRM, alarmHandler);

            int tries;
            while (tries < cpRetransmissions) {
                createWriteFrame(fd, A_SR, C_SET);
                alarm(delay);
                alarmFlag = FALSE;

                do {
                    if (read(fd, &byte, 1) > 0) {
                        determineSenderState(&state, byte);
                        printf("State: %d\n", state);
                    }
                } while (!alarmFlag && state != STOP);

                tries++;
                if (state == STOP) break;
            }
            if (state != STOP) {
                return -1;
            }

            break;
        }

        case LlRx: {
            do {
                if (read(fd, &byte, 1) > 0) {
                    determineReceiverState(&state, byte);
                    printf("State: %d\n", state);
                }
            } while (state != STOP);

            if (createWriteFrame(fd, A_SR, C_UA) < 0) return -1;

            break;
        }
        default:
            return -1;
            break;
    }
    return fd;
}

int makeConnection(const char *serialPort) {
    fd = open(serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPort);
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));

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

void determineSenderState(State *state, unsigned char byte) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == A_SR)
                *state = A_RCV;
            else if (byte == FLAG) {
            } else
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
            if (byte == (A_SR ^ C_UA))
                *state = BCC_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case BCC_OK:
            if (byte == FLAG)
                *state = STOP;
            else
                *state = START;
            break;
        default:
            break;
    }
}

void determineReceiverState(State *state, unsigned char byte) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            if (byte == A_SR)
                *state = A_RCV;
            else if (byte == FLAG) {
            } else
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
                *state = BCC_OK;
            else if (byte == FLAG)
                *state = FLAG_RCV;
            else
                *state = START;
            break;
        case BCC_OK:
            if (byte == FLAG)
                *state = STOP;
            else
                *state = START;
            break;
        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////
//                            LLWRITE                                  //
/////////////////////////////////////////////////////////////////////////
/*
int llwrite(const unsigned char *buf, int bufSize)
{
    int tramaSize = bufSize + 6;
    unsigned char *trama = (unsigned char *)malloc(tramaSize);
    trama[0] = FLAG;
    trama[1] = A_SR;
    trama[2] = C_N(tramaT);
    trama[3] = trama[1] ^ trama[2];
    memcpy(trama + 4, buf, bufSize);


    unsigned char bbc_2 = buf[0];
    for (int i = 1; i < bufSize; i++)
        bbc_2 ^= buf[i];
    int j = 4;
    // Byte stuffing
    for (int i = 4; i < tramaSize - 1; i++)
    {
        if (buf[i] == FLAG || buf[i] == ESC_KEY)
        {
            trama = (unsigned char *)realloc(trama, ++tramaSize);
            trama[j++] = ESC_KEY;
        }
        trama[j++] = buf[i];
    }
    trama[j++] = bbc_2;
    trama[j++] = FLAG;
    int transCount = 0;
    int reject = 0;
    int accept = 0;
    while (transCount < cpRetransmissions)
    {
        alarmFlag = FALSE;
        alarm(delay);
        reject = 0;
        accept = 0;
        while (alarmFlag = FALSE && !reject && !accept)
        {
            write(fd, trama, tramaSize);
            unsigned char r = readControlFrame(fd);
            if (!r)
            {
                continue;
            }
            else if (r = C_REJJ(0) || r == C_REJJ(1))
            {
                reject = 1;
            }
            else if (r == C_RRR(0) || r == C_RRR(1))
            {
                accept = 1;
                tramaT = (tramaT + 1) % 2;
            }
            else
                continue;
        }
        if (accept)
            break;
        transCount++;
    }
    free(trama);
    if (accept)
        return tramaSize;
    else
    {
        llclose(0);
        return -1;
    }
}

unsigned char processByte(unsigned char byte, State *state, unsigned char *c)
{

    switch (*state)
    {
    case START:
        if (byte == FLAG)
        {
            *state = FLAG_RCV;
        }
        break;
    case FLAG_RCV:
        if (byte == A_RS)
        {
            *state = A_RCV;
        }
        else if (byte != FLAG)
        {
            *state = START;
        }
        break;
    case A_RCV:
        if (byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1 || byte
== DISC)
        {
            *state = C_RCV;
            *c = byte;
        }
        else
        {
            *state = (byte == FLAG) ? FLAG_RCV : START;
        }
        break;
    case C_RCV:
        if (byte == (A_RS ^ *c))
        {
            *state = BCC_OK;
        }
        else
        {
            *state = (byte == FLAG) ? FLAG_RCV : START;
        }
        break;
    case BCC_OK:
        *state = (byte == FLAG) ? STOP : START;
        break;
    default:
        break;
    }
    return *c;
}

unsigned char interpretControlPacket(int fd)
{
    unsigned char byte, c = 0;
    State state = START;

    while (state != STOP && !alarmFlag)
    {
        if (read(fd, &byte, 1) > 0)
        {
            c = processByte(byte, &state, &c);
        }
        else
        {
        }
    }
    return c;
}

/////////////////////////////////////////////////////////////////////////
//                            LLREAD                                   //
/////////////////////////////////////////////////////////////////////////
int llread(unsigned char *packet)
{

    unsigned char byte, cField;
    int i = 0;
    State state = START;

    while (state != STOP)
    {
        if (read(fd, &byte, 1) > 0)
        {
            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == A_SR)
                    state = A_RCV;
                else if (byte != FLAG)
                    state = START;
                break;
            case A_RCV:
                if (byte == C_N(0) || byte == C_N(1))
                {
                    state = C_RCV;
                    cField = byte;
                }
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else if (byte == DISC)
                {
                    emitSupervisionSignal(fd, A_RS, DISC);
                    return 0;
                }
                else
                    state = START;
                break;
            case C_RCV:
                if (byte == (A_SR ^ cField))
                    state = DATA_READ;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case DATA_READ:
                if (byte == ESC_KEY)
                    state = DATA_ESC;
                else if (byte == FLAG)
                {
                    unsigned char bcc2 = packet[i - 1];
                    i--;
                    packet[i] = '\0';
                    unsigned char acc = packet[0];

                    for (unsigned int j = 1; j < i; j++)
                        acc ^= packet[j];

                    if (bcc2 == acc)
                    {
                        state = STOP;
                        createWriteFrame(fd, A_RS, C_RRR(tramaR));
                        tramaR = (tramaR + 1) % 2;
                        return i;
                    }
                    else
                    {
                        printf("Error: retransmition\n");
                        createWriteFrame(fd, A_RS, C_REJJ(tramaR));
                        return -1;
                    };
                }
                else
                {
                    packet[i++] = byte;
                }
                break;
            case DATA_ESC:
                state = DATA_READ;
                if (byte == ESC_KEY || byte == FLAG)
                    packet[i++] = byte;
                else
                {
                    packet[i++] = ESC_KEY;
                    packet[i++] = byte;
                }
                break;
            default:
                break;
            }
        }
    }
    return -1;
}
*/

/////////////////////////////////////////////////////////////////////////
//                            LLCLOSE                                  //
/////////////////////////////////////////////////////////////////////////

int llclose(int showStatistics) {
    State state = START;
    alarmCounter = 0;
    alarmFlag = FALSE;
    int STOP = FALSE;

    switch (role_) {
        case LlTx:

            (void)signal(SIGALRM, alarmHandler);

            while (alarmCounter < cpRetransmissions && state != STOP) {
                if (alarmFlag == FALSE) {
                    createWriteFrame(fd, A_SR, DISC);
                    alarm(delay);
                    alarmFlag = TRUE;

                    unsigned char byte;
                    while (state != STOP) {
                        read(fd, &byte, 1);
                        switch (state) {
                            case START:
                                if (byte == FLAG) state = FLAG_RCV;
                                break;
                            case FLAG_RCV:
                                if (byte == FLAG)
                                    state = FLAG_RCV;
                                else if (byte == A_RS)
                                    state = A_RS;
                                else
                                    state = START;
                                break;
                            case A_RCV:
                                if (byte == FLAG)
                                    state = FLAG_RCV;
                                else if (byte == DISC)
                                    state = C_RCV;
                                else
                                    state = START;
                                break;
                            case C_RCV:
                                if (byte == FLAG)
                                    state = FLAG_RCV;
                                else if (byte == (A_RS ^ DISC))
                                    state = BCC_OK;
                                else
                                    state = START;
                                break;
                            case BCC_OK:
                                if (byte == FLAG)
                                    state = STOP;
                                else
                                    state = START;
                                break;
                            default:
                                break;
                        }
                    }
                }
            }

            createWriteFrame(fd, A_SR, DISC);

            break;

        case LlRx:;
            unsigned char byte;
            while (state != STOP) {
                read(fd, &byte, 1);
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == A_SR)
                            state = A_RCV;
                        else
                            state = START;
                        break;
                    case A_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == DISC)
                            state = C_RCV;
                        else
                            state = START;
                        break;
                    case C_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == (A_SR ^ DISC))
                            state = BCC_OK;
                        else
                            state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG)
                            state = STOP;
                        else
                            state = START;
                        break;
                    default:
                        break;
                }
            }

            createWriteFrame(fd, A_RS, DISC);

            state = 0;
            alarmCounter = 0;
            alarmFlag = FALSE;

            while (state != STOP) {
                read(fd, &byte, 1);
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;

                        break;
                    case FLAG_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == A_SR)
                            state = A_RCV;
                        else
                            state = START;
                        break;
                    case A_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == C_UA)
                            state = C_RCV;
                        else
                            state = START;
                        break;
                    case C_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == (A_SR ^ C_UA))
                            state = BCC_OK;
                        else
                            state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG)
                            state = STOP;
                        else
                            state = START;
                        break;
                    default:
                        break;
                }
            }

            break;
        default:
            break;
    }

    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    printf("Connection closed\n");
    return 0;
}

int createWriteFrame(int fd, unsigned char A, unsigned char C) {
    unsigned char frame[5] = {FLAG, A, C, A ^ C, FLAG};
    return write(fd, frame, 5);
}
