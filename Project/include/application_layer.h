// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include <stdio.h>
#include <stdlib.h>

#define DATA_PAYLOAD 1

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

unsigned char *buildControlPacket(unsigned int controlFieldValue,
                                  const long int fileSize, const char *filename,
                                  int *cpSize);

unsigned char *processControlPacket(unsigned char *packet, int readSize,
                                    unsigned long int *fileSize);

#endif  // _APPLICATION_LAYER_H_
