#ifndef MACROS_H
#define MACROS_H

#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#define FALSE 0
#define TRUE 1

#define SET_SIZE 5
#define UA_SIZE 5

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1  // POSIX compliant source

#define FLAG 0x7E

#define A_SR 0x03
#define A_RS 0x01

#define C_SET 0x03
#define C_UA 0x07

#define BCC(a,c) (a ^ c)

#define BUF_SIZE 256

#define TIMEOUT 3
#define N_TRIES 3


typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    STOP_STATE,
} State;

#endif