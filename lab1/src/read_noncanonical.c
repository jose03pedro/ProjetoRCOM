// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "../include/macros.h"

volatile int STOP = FALSE;

void stateMachine(enum State *state, int *a, int *c, char byte)
{

    switch (*state)
    {
    case START:
        if (byte == FLAG)
            *state = FLAG_RCV;
        break;

    case FLAG_RCV:
        if (byte == A_SR)
        {
            *state = A_RCV;
            *a = byte;
        }
        else if (byte != FLAG)
            *state = START;
        break;

    case A_RCV:
        if (byte == FLAG)
            *state = FLAG_RCV;
        if (byte == C_UA)
        {
            *state = C_RCV;
            *c = byte;
        }
        else
            *state = START;
        break;

    case C_RCV:
        if (byte == FLAG)
            *state = FLAG;
        if (byte == (*a ^ *c))
            *state = BCC_OK;
        else
            *state = START;
        break;

    case BCC_OK:
        if (byte == FLAG)
            *state = STOP_STATE;
        else
            *state = START;
        break;

    case STOP_STATE:
        break;
    }
}

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

    // Open serial port device for reading and writing and not as controlling
    // tty because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
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
    newtio.c_cc[VTIME] = 0;  // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;   // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Loop for input
    unsigned char buf[BUF_SIZE + 1] = {
        0};  // +1: Save space for the final '\0' char

    while (STOP == FALSE) {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, SET_SIZE);

        for (int i = 0; i < bytes; i++) {
            printf("var = 0x%02X\n", buf[i]);
        }
        if (bytes == SET_SIZE && buf[0] == FLAG && buf[1] == A_SR && buf[2] == C_SET && buf[3] == (A_SR ^ C_SET) && buf[4] == FLAG) {
            buf[0] = FLAG;
            buf[1] = A_SR;
            buf[2] = C_UA;
            buf[3] = A_SR ^ C_UA;
            buf[4] = FLAG;
            printf("Received SET frame.\n");
        }

        buf[bytes] = '\0';  // Set end of string to '\0', so we can printf

        write(fd, buf, UA_SIZE);

        // printf(":%s:%d\n", buf, bytes);
        if (buf[4] == 0x7E) STOP = TRUE;
    }

    // The while() cycle should be changed in order to respect the
    // specifications of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
