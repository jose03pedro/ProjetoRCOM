#ifndef AUXILIARY_FUNCTIONS_H
#define AUXILIARY_FUNCTIONS_H

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define FALSE 0
#define TRUE 1

#define SET_SIZE 5
#define UA_SIZE 5

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE 38400
#define _POSIX_SOURCE 1  // POSIX compliant source

#define FLAG 0x7E
#define ESCAPE 0x7D
#define STUFFING 0x20
#define ESCAPE_FLAG (FLAG ^ STUFFING)
#define ESCAPE_ESCAPE (ESCAPE ^ STUFFING)

#define A_SR 0x03
#define A_RS 0x01

#define C_DISC 0x0B
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_I0 0x00
#define C_I1 0x40

#define BCC(a, c) (a ^ c)

#define BUF_SIZE 256

#define TIMEOUT 3
#define N_TRIES 3

extern unsigned char txFrame;
extern unsigned char rxFrame;

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    READING_DATA,
    STOP_STATE,
    ESC_FOUND,
} State;

void stateMachine(unsigned char byte, State *state);

void stateMachineTx(unsigned char byte, State *state);

int stateMachinePck(unsigned char byte, State *state, unsigned char *packet,
                    int fd);

int transmitFrame(unsigned char A, unsigned char C, int fd);

int openConnection(const char *serialPort);

void sendControlPackets(int fd, const char *filename, int fileSize, unsigned char sequence);

#endif