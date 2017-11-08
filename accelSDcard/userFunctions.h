#ifndef userFunctions_h
#define userFunctions_h
/** User data types and functions
  *
  */
  
// Prototypes
#define BUFFER_SIZE 128

#include "SD.h"
#include "Arduino.h"

enum t_axisOrder {
  X_AXIS, // 0
  Y_AXIS, // 1
  Z_AXIS  // 2
};

const unsigned long HoursInMicros = 3600000000;

// Global variables
//String measurement = "";
struct Acceleration{
  // Raw data
  unsigned long t;
  short ax;
  short ay;
  short az;
};

void readData(Acceleration*);
void readData(unsigned char*);
void printData(Acceleration*);
void printData(unsigned char*);
void printHeader(File);
#endif // userFunctions_h
