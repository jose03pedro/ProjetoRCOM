// Link layer protocol implementation

#include "../include/link_layer.h"
#include "../include/auxiliary_functions.h"

int alarmCount = 0;
int alarmEnabled = FALSE;

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

    if (role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);

        do {
            transmitFrame(fd, A_SR, C_SET);
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

            retransmissions--;
        } while (retransmissions != 0 && state != STOP_STATE);

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

        transmitFrame(fd, A_RS, C_UA);
    } else {
        return -1;
    }

    return 1;
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
    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    State state = START;
    unsigned char rByte;
    (void) signal(SIGALRM, alarmHandler);
    

    while (retransmissions > 0 && state != STOP_STATE)
    {
        alarmCount++;
        alarm(timer);
        alarmEnabled = FALSE;

        while(state != STOP_STATE && alarmEnabled == FALSE)
        {
            switch (read(showStatistics, &rByte, 1)) {
                case 1:
                    stateMachine(rByte, &state);
                    if(state == STOP_STATE) {
                        printf("Connection established\n");
                    }
                    else return 1;
                    break;
                case -1:
                    return -1;
                    break;
                default:
                    break;
            }
        }
        retransmissions--;
    }

    
    transmitFrame(showStatistics, A_SR, C_UA);
    if(close(showStatistics) == -1) return -1;
    else return 1;
}
