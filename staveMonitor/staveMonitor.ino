/** Dino Tahirovic
 *  QMUL
 *  28/9/2017
 */

#include <SparkFunMPU9250-DMP.h>
#include <MPU9250_RegisterMap.h>

#define SerialPort SerialUSB

MPU9250_DMP mpu;

void setup() {
  
  SerialPort.begin(115200);

  if (mpu.begin() != INV_SUCCESS)
    { 
      SerialPort.println("MPU9520 not available");
      while(1);
    }

  delay(3000);

   unsigned char c;
    mpu_read_reg(MPU9250_WHO_AM_I, &c);
    SerialPort.print("MPU9520 Who am I returns 0x"); SerialPort.println(c, HEX);

      if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    SerialPort.println("MPU9250 is online...");
    
    unsigned short rate = mpu.getSampleRate();
    SerialPort.println("Sample rate obtained: ");
    SerialPort.println(rate);

    
  }
  
   mpu.setSensors(INV_XYZ_ACCEL);
   mpu.setAccelFSR(2);
   mpu.setSampleRate(1000);
   
}

void loop() {

  if (mpu.dataReady())
  {
    mpu.updateAccel();
    SerialPort.print(mpu.measurement);
  }
  //if (mpuLog.length() > SD_LOG_WRITE_BUFFER_SIZE)
  
}

