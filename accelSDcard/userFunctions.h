#ifndef userFunctions_h
#define userFunctions_h
/** User data types and functions
  * 08/11/2017 Dino Tahirovic
  */
  
// Prototypes
#include "SD.h"
#include "config.h"
#include "Arduino.h"

enum t_axisOrder {
  X_AXIS, // 0
  Y_AXIS, // 1
  Z_AXIS  // 2
};

const unsigned long HoursInMicros = 3600000000;

void readData(unsigned char*);
void printData(unsigned char[BUFFER_SIZE][BINARY_STRING]);
void printHeader(unsigned short);
#endif // userFunctions_h
