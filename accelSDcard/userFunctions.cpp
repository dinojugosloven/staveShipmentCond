/** User functions
  * dino Tahirovic, QMUL 8/11/2017
  */

#include "userFunctions.h"
#include "config.h"
#include "MPU9250_RegisterMap.h"
extern "C" {
#include "util/arduino_mpu9250_i2c.h"
}

// Global vars defined in .ino file
extern unsigned long currentTime; // in millis
extern File logFile;
extern unsigned short dataPointer;

// implementations  
int readData(unsigned char* binaryData)
{
  unsigned long timeMillis = millis();
  binaryData[0] = (timeMillis >> 24) & 0xFF; // 31-24
  binaryData[1] = (timeMillis >> 16) & 0xFF; // 23-16
  binaryData[2] = (timeMillis >> 8) & 0xFF;  // 15-8
  binaryData[3] = timeMillis & 0xFF;         //  7-0
  
  if (i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, &binaryData[4]))
    return(-1);
    
  return(0);
}

void printData(unsigned char binaryBuffer[BUFFER_SIZE][BINARY_STRING]){


  for (unsigned int i=0; i<dataPointer; i++){
    if (sdCardPresent){
      logFile.write(binaryBuffer[i], 10);
    } else {
      
    }
    
  }
  
  logFile.flush(); 
}

void printHeader(unsigned short _aSense)
{
  logFile.print("Sensitivity = ");
  logFile.print(_aSense);
	logFile.println(" t ax ay az : 4,2,2,2 B");
  logFile.flush();
	return;
}
