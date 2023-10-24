// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_
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

#include "application_layer.h"

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
#define STUF_FLAG 0x5e
#define STUF_ESCAPE 0x5d

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

#define BUF_SIZE 1000

typedef enum {
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct {
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

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


int transmitFrame(unsigned char A, unsigned char C);

int openConnection(const char *serialPort);

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console
// on close. Return "1" on success or "-1" on error.
int llclose(int showStatistics);

#endif  // _LINK_LAYER_H_
