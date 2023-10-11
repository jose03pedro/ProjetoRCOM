// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define FALSE 0
#define TRUE 1

#define SET_SIZE 5

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1  // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256

volatile int STOP = FALSE;

int main(int argc, char *argv[]) {
    (void)signal(SIGALRM, alarmHandler);

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
    newtio.c_cc[VTIME] = 0;  // Timeout entre bytes
    newtio.c_cc[VMIN] = 0;   // Numero minimo de bytes a serem lidos

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

    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

    buf[0] = 0x7e;
    buf[1] = 0x03;
    buf[2] = 0x03;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = 0x7e;

    do {
        alarmCount++;
        int res = write(fd, buf, SET_SIZE);  // envia o \0
        printf("%d bytes written\n", res);

        alarm(3);
        alarmEnabled = TRUE;

        // state = Start;
        // unsigned char checkBuf[2]; // checkBuf terá valores de A e C para
        // verificar BCC

        while (STOP == FALSE) {     /* loop for input */
            res = read(fd, buf, 1); /* returns after 1 char has been input */
            if (res == -1) break;

            printf("nº bytes lido: %d - ", res);
            printf("content: %#4.2x\n", buf[0]);

            // determineState(&state, checkBuf, buf1[0]);

            if (/*state == DONE || */ alarmEnabled) STOP = TRUE;
        }

    } while (alarmCount < 3 && alarmEnabled);

    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written\n", bytes);

    // Set alarm function handler

    while (alarmCount < 4) {
        if (alarmEnabled == FALSE) {
            alarm(3);  // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
    }

    /*
    int bytes1 = read(fd,buf,BUF_SIZE);

        for(int i = 0; i<bytes1;i++){
            printf("var = 0x%02x\n" buf[i]);
        }
        */

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
