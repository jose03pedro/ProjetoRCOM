#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "../include/macros.h"

volatile int STOP = FALSE;

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

/*
void stateMachine(enum State *state, int *a, int *c, char byte) {
    switch (*state) {
        case START:
            if (byte == FLAG) {
                *state = FLAG_RCV;
            }
            break;

        case FLAG_RCV:
            if (byte == FLAG) {
                *state = FLAG_RCV;
            } else if (byte == A_SR) {
                *state = A_RCV;
                *a = byte;
            } else {
                *state = START;
            }
            break;

        case A_RCV:
            if (byte == FLAG) {
                *state = FLAG_RCV;
            } else if (byte == C_SET) {
                *state = C_RCV;
                *c = byte;
            } else {
                *state = START;
            }
            break;

        case C_RCV:
            if (byte == FLAG) {
                *state = FLAG_RCV;
            } else if (byte == (*a ^ *c)) {
                *state = BCC_OK;
            } else {
                *state = START;
            }
            break;

        case BCC_OK:
            if (byte == FLAG) {
                *state = STOP;
            } else {
                *state = START;
            }
            break;

        case STOP:
            // Handle STOP state if needed
            break;
    }
} 
*/


int main(int argc, char *argv[]) {

    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2) {
        printf(
            "Incorrect program usage\n"
            "Usage: %s <SerialPort>\n"
            "Example: %s /dev/ttyS1\n",
            argv[0], argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling
    // tty because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    (void)signal(SIGALRM, alarmHandler);

    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;  // Timeout between bytes
    newtio.c_cc[VMIN] = 1;   // Minimum number of bytes to be read

    // VTIME and VMIN should be changed to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    // TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

    buf[0] = FLAG;
    buf[1] = A_SR;
    buf[2] = C_SET;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    int res;

    while (alarmCount < N_TRIES) {
        res = write(fd, buf, SET_SIZE);

        if (res == -1) {
            printf("Write failed\n");
        } else {
            printf("%d bytes written\n", res);
        }

        alarmCount++;
        alarm(TIMEOUT);
        alarmEnabled = TRUE;

        //STOP = FALSE;

        //enum State state = START;

        // Reset buf for reception
        memset(buf, 0, BUF_SIZE);

        // Receive response
        res = read(fd, buf, UA_SIZE);
        if (res == -1) {
            printf("Read failed\n");
            break;
        } else if (res > 0) {
            printf("Received %d bytes: \n", res);

            for (int i = 0; i < res; i++) {
                printf("var = 0x%02X\n", buf[i]);
            }

// Check if the received bytes match the UA frame structure
            if (res == UA_SIZE && buf[0] == FLAG && buf[1] == A_SR &&
                buf[2] == C_UA && buf[3] == (A_SR ^ C_UA) && buf[4] == FLAG) {
                printf("Received UA frame.\n");
                alarm(0);
                alarmEnabled = FALSE;
                break;
            } else {
                printf("Received frame does not match UA.\n");
                // Handle other frames if needed
            }

            printf("\n");

            // Reset the alarm and stop receiving
            alarm(0);
            alarmEnabled = FALSE;
            /*if (state == STOP) {
                break;
            }*/
        }









    }
    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
