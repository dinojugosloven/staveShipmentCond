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
    //for (int iByte = 0; iByte < 6; iByte++){
      //binaryData[4+iByte] = tmp[iByte];
    //}
  }
  dataPointer++;
}
void printData(unsigned char* binaryData){

  logFile.write(binaryData, 10);

  //Debug
  short ax = (binaryData[4] << 8) | binaryData[5];
  short ay = (binaryData[6] << 8) | binaryData[7];
  short az = (binaryData[8] << 8) | binaryData[9];
  String outputString = String(0) + "," + 
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
	logFile.println("t,ax,ay,az");
	return;
}
