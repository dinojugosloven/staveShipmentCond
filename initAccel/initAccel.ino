/** Dino Tahirovic
 *  QMUL
 *  28/9/2017
 */

#include <SparkFunMPU9250-DMP.h>
#include <MPU9250_RegisterMap.h>

#define com SerialUSB

MPU9250_DMP mpu;

void setup() 
{
  com.begin(115200);

  if (mpu.beginAccel() != INV_SUCCESS)
    { 
      com.println("MPU9520 Accelerator not available");
      com.println("Ctrl+Z should finish the program.");
      while(1);
    }

  delay(3000);

  unsigned char c;
  mpu_read_reg(MPU9250_WHO_AM_I, &c);
  com.print("MPU9520 Who am I returns 0x"); com.println(c, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71 for MPU9250
  {
    com.println("MPU9250 is online...");
    
    mpu.setSampleRate(500);
    unsigned short rate = mpu.getSampleRate();
    com.println("Sample rate obtained: ");
    com.println(rate);
    
    com.print("Accel sensitivity, LSB value 1/"); com.println(mpu.getASense());
    mpu.setAccelFSR(16);
    com.print("Accel FSR "); com.println(mpu.getAccelFSR());

    com.println("Reg     Value");
    // General configuration
    unsigned char registerByte;
    mpu_read_reg(MPU9250_CONFIG, &registerByte);
    com.print(MPU9250_CONFIG,HEX);com.print("     ");com.println(registerByte,BIN);

    // Power managment 1
    unsigned char microReg = MPU9250_PWR_MGMT_1;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("     ");com.println(registerByte,BIN);

    // Power management 2
    microReg = MPU9250_PWR_MGMT_2;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("     ");com.println(registerByte,BIN);
  
  } // MPU9250 check loop
}

void loop() {

 /* if (mpu.dataReady())
  {
    mpu.updateAccel();
    com.print(mpu.measurement);
  }*/
  //if (mpuLog.length() > SD_LOG_WRITE_BUFFER_SIZE)
  
}

