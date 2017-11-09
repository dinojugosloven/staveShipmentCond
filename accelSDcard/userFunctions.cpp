/** User functions
  * dino Tahirovic, QMUL 8/11/2017
  */

#include "userFunctions.h"
#include "config.h"
#include "MPU9250_RegisterMap.h"
extern "C" {
#include "util/arduino_mpu9250_i2c.h"
}

extern unsigned long currentTime; // in millis
extern File logFile;
extern unsigned short dataPointer;

// implementations  
void readData(Acceleration* rawData)
{


    unsigned char tmp[6];
	
	// read accel data directly from MPU9250 register
    if (!i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, tmp))
    {
      // sprintf is too expensive, 800 us because of formating
      //int writeStatus = sprintf(rawData, "%lu, %hd, %hd, %hd \n", 
        rawData[dataPointer].t = currentTime>>10; //curent time divided by 1024
        rawData[dataPointer].ax = (tmp[0] << 8) | tmp[1];
        rawData[dataPointer].ay = (tmp[2] << 8) | tmp[3];
        rawData[dataPointer].az = (tmp[4] << 8) | tmp[5];   
    };
    dataPointer++;

}

void readData(unsigned char* binaryData)
{
  //unsigned char tmp[6];
  if (!i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, &binaryData[4]))
  {
    unsigned long timeMillis = currentTime>>10;
    binaryData[0] = (timeMillis >> 24) & 0xFF; // 31-24
    binaryData[1] = (timeMillis >> 16) & 0xFF; // 23-16
    binaryData[2] = (timeMillis >> 8) & 0xFF;  // 15-8
    binaryData[3] = timeMillis & 0xFF;         //  7-0
  }
  dataPointer++;
}
void printData(unsigned char* binaryData){

  logFile.write(binaryData, 10);

  //Debug
  unsigned long time = binaryData[0]<<24;
  time |= binaryData[1]<<16;
  time |= binaryData[2]<<8;
  time |= binaryData[3];
  short ax = (binaryData[4] << 8) | binaryData[5];
  short ay = (binaryData[6] << 8) | binaryData[7];
  short az = (binaryData[8] << 8) | binaryData[9];
  String outputString = String(time) + "," + 
      String(ax) + "," + 
      String(ay) + "," +
      String(az);
  com.println(outputString);
  //
  
  dataPointer = 0;
}

// Creating non-binary file
// With String
// or char[]
// if too slow, use binary file
void printData(Acceleration* rawData){

  
}

void printHeader(File logFile)
{
	logFile.println("t,ax,ay,az : 4,2,2,2 B");
	return;
}
