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
extern File logFile;
extern bool sdCardPresent;
extern unsigned short dataPointer;

// implementations  
int readData(unsigned char* binaryData)
{
  unsigned long timeStamp = micros();
  binaryData[0] = (timeStamp >> 24) & 0xFF; // 31-24
  binaryData[1] = (timeStamp >> 16) & 0xFF; // 23-16
  binaryData[2] = (timeStamp >> 8) & 0xFF;  // 15-8
  binaryData[3] = timeStamp & 0xFF;         //  7-0
  
  if (i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, &binaryData[4]))
    return(-1);
    
  return(0);
}

void printData(unsigned char binaryBuffer[BUFFER_SIZE][BINARY_STRING]){

    for (unsigned int i=0; i<dataPointer; i++){
      if (sdCardPresent) {
        logFile.write(binaryBuffer[i], 10);
      } 
      else {
      unsigned long time = binaryBuffer[i][0]<<24;
      time |= binaryBuffer[i][1]<<16;
      time |= binaryBuffer[i][2]<<8;
      time |= binaryBuffer[i][3];
      short ax = (binaryBuffer[i][4] << 8) | binaryBuffer[i][5];
      short ay = (binaryBuffer[i][6] << 8) | binaryBuffer[i][7];
      short az = (binaryBuffer[i][8] << 8) | binaryBuffer[i][9];
      String outputString = String(time) + "," + 
        String(ax) + "," + 
        String(ay) + "," +
        String(az);
      com.println(outputString);
    
    }
    
  }
  if (sdCardPresent) logFile.flush();
   
}

void printHeader(unsigned short _aSense)
{
  logFile.print("Sensitivity = ");
  logFile.print(_aSense);
	logFile.println(" t ax ay az : 4,2,2,2 B");
  logFile.flush();
	return;
}
