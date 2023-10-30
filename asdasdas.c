#include "link_layer.h"

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
int frameCounter = 0;

int nRetransmissions, timeout, baudRate, fd, role;

struct termios oldtio, newtio;

void alarmHandler(int signal)
{
    alarmCount++;
    alarmEnabled = TRUE;
    printf("Alarm #%d\n", alarmCount);
}

int llopen(LinkLayer connectionParameters)
{

    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    role = connectionParameters.role;
    baudRate = connectionParameters.baudRate;

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("Error opening serial port");
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    State state = START;
    unsigned char byte;

    switch (role)
    {
    case LlRx:
    {

        while (state != STOP)
        {
            unsigned char bytes_read = read(fd, &byte, BUFFER_SIZE);
            if (bytes_read > 0)
                rxStateMachine(&state, byte);
        }

        if (createWriteFrame(A_RS, C_UA) < 0)
            return -1;
        break;
    }

    case LlTx:
    {
        (void)signal(SIGALRM, alarmHandler);

        while (nRetransmissions > 0 && state != STOP)
        {

            if (createWriteFrame(A_SR, C_SET) < 0)
                return -1;

            alarm(timeout);
            alarmEnabled = FALSE;

            while (!alarmEnabled && state != STOP)
            {
                if (read(fd, &byte, BUFFER_SIZE) > 0)
                    txStateMachine(&state, byte);
            }
            nRetransmissions--;
        }
        nRetransmissions = connectionParameters.nRetransmissions;

        if (state != STOP)
            return -1;
        break;
    }

    default:
        return -1;
    }
    return fd;
}

unsigned int rxStateMachine(State *state, unsigned char byte)
{

    switch (*state)
    {
    case START:
        if (byte == FLAG)
            *state = FLAG_RCV;
        break;
    case FLAG_RCV:
        if (byte == A_SR)
            *state = A_RCV;
        else if (byte == FLAG)
            *state = FLAG_RCV;
        else
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
            *state = STOP;
        else
            *state = START;
        break;
    default:
        break;
    }

    return 0;
}

unsigned int txStateMachine(State *state, unsigned char byte)
{

    switch (*state)
    {
    case START:
        if (byte == FLAG)
            *state = FLAG_RCV;
        break;
    case FLAG_RCV:
        if (byte == A_RS)
            *state = A_RCV;
        else if (byte == FLAG)
            *state = FLAG_RCV;
        else
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
            *state = STOP;
        else
            *state = START;
        break;
    default:
        break;
    }

    return 0;
}

int llwrite(const unsigned char *dataBuffer, int bufferSize)
{

    int totalFrameSize = bufferSize + 6;
    unsigned char *transmissionFrame = (unsigned char *)malloc(totalFrameSize);
    transmissionFrame[0] = FLAG;
    transmissionFrame[1] = A_SR;
    transmissionFrame[2] = (frameCounter == 0) ? 0x00 : 0x40;
    transmissionFrame[3] = transmissionFrame[1] ^ transmissionFrame[2];

    memcpy(transmissionFrame + 4, dataBuffer, bufferSize);

    unsigned char bccValue = dataBuffer[0];
    for (int idx = 1; idx < bufferSize; idx++)
        bccValue ^= dataBuffer[idx];

    int framePosition = 4;
    for (unsigned int idx = 0; idx < bufferSize; idx++)
    {
        if (dataBuffer[idx] == FLAG || dataBuffer[idx] == ESC_BYTE)
        {
            transmissionFrame = realloc(transmissionFrame, ++totalFrameSize);
            transmissionFrame[framePosition++] = ESC_BYTE;
            transmissionFrame[framePosition++] = dataBuffer[idx] ^ 0x20;
        }
        else
        {
            transmissionFrame[framePosition++] = dataBuffer[idx];
        }
    }

    if (bccValue == FLAG || bccValue == ESC_BYTE)
    {
        transmissionFrame = realloc(transmissionFrame, ++totalFrameSize);
        transmissionFrame[framePosition++] = ESC_BYTE;
        transmissionFrame[framePosition++] = bccValue ^ 0x20;
    }
    else
    {
        transmissionFrame[framePosition++] = bccValue;
    }
    transmissionFrame[framePosition++] = FLAG;

    int attempts = 0;
    int isAccepted = 0;

    while (attempts < nRetransmissions && !isAccepted)
    {
        alarm(0);
        write(fd, transmissionFrame, framePosition);
        alarm(timeout);
        alarmEnabled = 0;

        unsigned char receivedResponse = llwriteStateMachine();

        if (receivedResponse == RR0 || receivedResponse == RR1)
        {
            isAccepted = 1;
            frameCounter = (receivedResponse == RR0) ? 0 : 1;
        }
        attempts++;
    }

    alarmCount = 0;
    free(transmissionFrame);

    return isAccepted;
}

int llread(unsigned char *packet)
{
    unsigned char byte, bcc1_calcV, bcc2_rcv, bcc2_calcV;
    unsigned int packet_it = 0;
    State state = START;

    while (state != STOP)
    {
        if (read(fd, &byte, BUFFER_SIZE) > 0)
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
                if ((byte == 0x00 && frameCounter == 0) || (byte == 0x40 && frameCounter == 1))
                {
                    frameCounter = (byte == 0x00) ? 1 : 0;
                    state = C_RCV;
                    bcc1_calcV = byte ^ A_SR;
                }
                else if (byte == DISC)
                {
                    createWriteFrame(A_RS, DISC);
                    return 0;
                }
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                if (byte == bcc1_calcV)
                    state = READ_DATA;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case READ_DATA:
                if (byte == ESC_BYTE)
                    state = ESC;
                else if (byte == FLAG)
                {
                    bcc2_rcv = packet[--packet_it];
                    packet[packet_it] = '\0';
                    bcc2_calcV = packet[0];
                    for (int j = 1; j < packet_it; j++)
                        bcc2_calcV ^= packet[j];

                    if (bcc2_calcV == bcc2_rcv)
                    {
                        state = STOP;
                        createWriteFrame(A_RS, (frameCounter == 0) ? 0x05 : 0x85);
                        return packet_it;
                    }
                    else
                    {
                        printf("Data Fail/Corrupted!\n");
                        createWriteFrame(A_RS, (frameCounter == 0) ? 0x01 : 0x81);
                        return -1;
                    }
                }
                else
                    packet[packet_it++] = byte;
                break;

            case ESC:
                state = READ_DATA;
                packet[packet_it++] = (byte == 0x5e) ? 0x7e : (byte == 0x5d) ? 0x7d
                                                                             : byte;
                break;

            default:
                break;
            }
        }
    }

    return -1;
}

int llclose(int showStatistics)
{
    State state = START;
    alarmCount = 0;
    alarmEnabled = FALSE;

    switch (role)
    {
    case LlTx:

        (void)signal(SIGALRM, alarmHandler);

        while (alarmCount < nRetransmissions && state != STOP)
        {
            if (!alarmEnabled)
            {
                createWriteFrame(A_SR, DISC);
                alarm(timeout);
                alarmEnabled = TRUE;

                unsigned char byte;
                while (state != STOP)
                {

                    read(fd, &byte, 1);

                    switch (state)
                    {
                    case START:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == FLAG)
                            state = FLAG_RCV;
                        else if (byte == A_RS)
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
                        else if (byte == (A_RS ^ DISC))
                            state = BCC1_OK;
                        else
                            state = START;
                        break;
                    case BCC1_OK:
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

        createWriteFrame(A_SR, C_UA);

        break;

    case LlRx:;
        unsigned char byte;
        while (state != STOP)
        {
            read(fd, &byte, 1);
            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
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
                    state = BCC1_OK;
                else
                    state = START;
                break;
            case BCC1_OK:
                if (byte == FLAG)
                    state = STOP;
                else
                    state = START;
                break;
            default:
                break;
            }
        }

        createWriteFrame(A_RS, DISC);

        state = 0;
        alarmCount = 0;
        alarmEnabled = FALSE;

        while (state != STOP)
        {
            read(fd, &byte, 1);
            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;

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
                    state = BCC1_OK;
                else
                    state = START;
                break;
            case BCC1_OK:
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

    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    printf("Connection closed\n");
    return 0;
}

int createWriteFrame(unsigned char a, unsigned char c)
{
    unsigned char buf[5] = {FLAG, a, c, a ^ c, FLAG};
    return write(fd, buf, 5);
}

unsigned int llwriteStateMachine()
{
    State currentState = START;
    unsigned char receivedByte;
    unsigned char controlByte = 0;

    while (!alarmEnabled)
    {
        unsigned char bytesRead = read(fd, &receivedByte, BUFFER_SIZE);

        if (bytesRead > 0)
        {
            switch (currentState)
            {
            case START:
                if (receivedByte == FLAG)
                    currentState = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (receivedByte == A_RS)
                    currentState = A_RCV;
                else if (receivedByte != FLAG)
                    currentState = START;
                break;

            case A_RCV:
                if (receivedByte == RR0 || receivedByte == RR1 || receivedByte == REJ0 || receivedByte == REJ1 || receivedByte == DISC)
                {
                    currentState = C_RCV;
                    controlByte = receivedByte;
                }
                else if (receivedByte == FLAG)
                    currentState = FLAG_RCV;
                else
                    currentState = START;
                break;

            case C_RCV:
                if (receivedByte == (A_RS ^ controlByte))
                    currentState = BCC1_OK;
                else if (receivedByte == FLAG)
                    currentState = FLAG_RCV;
                else
                    currentState = START;
                break;

            case BCC1_OK:
                if (receivedByte == FLAG)
                {
                    return controlByte;
                }
                else
                    currentState = START;
                break;

            default:
                break;
            }
        }
    }
    return 0;
}