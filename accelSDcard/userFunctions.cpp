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

// Creating non-binary file
// With String
// or char[]
// if too slow, use binary file
void printData(Acceleration* rawData){

    String serialOutput = String(rawData[0].t) + "," + 
      String(rawData[0].ax) + "," + 
      String(rawData[0].ay) + "," +
      String(rawData[0].az);
		//char dataStream[1*sizeof(Acceleration)];
    
    //size_t outputLength = serialOutput.length();
		logFile.write(serialOutput.c_str(), serialOutput.length());
   dataPointer = 0;
        //logFile.flush(); // flushing every time?
}

void printHeader(File logFile)
{
	logFile.println("t,ax,ay,az");
	return;
}
