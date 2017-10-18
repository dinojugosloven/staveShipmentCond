/** Dino Tahirovic
 *  QMUL
 *  28/9/2017
 */

#include <Wire.h>
#include <Arduino.h>
extern "C" {
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}
#include <MPU9250_RegisterMap.h>

#define com SerialUSB

// global types & constants
#define SD_LOG_WRITE_BUFFER_SIZE 64
#define BUFFER_SIZE 16
#define MAX_BUFFER_LENGTH 15

typedef int inv_error_t;
#define INV_SUCCESS 0
#define INV_ERROR 0x20
enum t_axisOrder {
  X_AXIS, // 0
  Y_AXIS, // 1
  Z_AXIS  // 2
};

// Global variables
//String measurement = "";
unsigned long accelBuffer[BUFFER_SIZE];
unsigned short dataPointer; // how large is short - 8 bits? only 256 then

void setup() 
{
  com.begin(115200);

  dataPointer = 0;

  // MPU::Begin
  inv_error_t result;
  struct int_param_s int_param;
  Wire.begin(); //communication between ARM Cortex and MPU9250?
  result = mpu_init(&int_param);

  if (result != INV_SUCCESS)
    { 
      com.println("MPU9520 Accelerator not available");
      com.println("Ctrl+Z should finish the program.");
      while(1);
    }
  // Init accelerometer
  mpu_set_bypass(1); // Place all slaves (including compass) on primary bus
  mpu_set_sensors(INV_XYZ_ACCEL);
  mpu_set_sample_rate(1000);
  delay(3000);

  // Probe the sensor
  unsigned char c;
  mpu_read_reg(MPU9250_WHO_AM_I, &c);
  
  com.print("MPU9520 Who am I returns 0x"); com.println(c, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71 for MPU9250
  {
    com.println("MPU9250 is online...");
    
    //mpu.setSampleRate(500);
    //unsigned short rate = mpu.getSampleRate();
    //com.println("Sample rate obtained: ");
    //com.println(rate);
    
    //com.print("Accel sensitivity, LSB value 1/"); com.println(mpu.getASense());
    //mpu.setAccelFSR(16);
    //com.print("Accel FSR "); com.println(mpu.getAccelFSR());

    com.println("Reg (HEX)   Value (BIN)");
    // General configuration
    unsigned char registerByte;
    mpu_read_reg(MPU9250_CONFIG, &registerByte);
    com.print(MPU9250_CONFIG,HEX);com.print("         ");com.println(registerByte,BIN);

    //Accelerometer configuration
    mpu_set_accel_fsr(16);
    unsigned char microReg = MPU9250_ACCEL_CONFIG;
    mpu_read_reg(microReg, &registerByte);
    com.print("FSR [g]");com.print("        ");com.println(registerByte,DEC);

    microReg = MPU9250_ACCEL_CONFIG_2;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("        ");com.println(registerByte,BIN);

    // I2C master clock frequency
    microReg = MPU9250_I2C_MST_CTRL;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("        ");com.println(registerByte,BIN);
    
    // Power managment 1
     microReg = MPU9250_PWR_MGMT_1;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("        ");com.println(registerByte,BIN);

    // Power management 2
    microReg = MPU9250_PWR_MGMT_2;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("        ");com.println(registerByte,BIN);
  
  } // MPU9250 check loop

}

void loop() {

  unsigned char intStatusReg;
  if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
  {
    unsigned int statusReady = (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
    
    // read accel data
    short data[3];
    unsigned long timeStamp;
    
    if ( !mpu_get_accel_reg(data, &timeStamp) )
    {
      //com.print(micros()/1000.0); com.print(" ");com.print(timeStamp); com.println(" "); 
      //com.print(data[X_AXIS]); com.print(" "); com.print(data[Y_AXIS]); com.print(" "); com.println(data[Z_AXIS]);
      accelBuffer[dataPointer] = timeStamp;
      dataPointer++;
    }
  } // INT_STATUS ready
  
  if (dataPointer == MAX_BUFFER_LENGTH) {
    //buffer full
    for (int i=0; i<MAX_BUFFER_LENGTH; i++) {
      com.print(i);com.print(" ");
      com.print(accelBuffer[i]); com.println(" ms");
    }
    dataPointer = 0;
  }
  
  /*
  if (measurement.length() > SD_LOG_WRITE_BUFFER_SIZE){
    com.print(measurement);
    measurement="";
  }*/
  
}

