/** Dino Tahirovic
 *  QMUL
 *  28/9/2017
 */

#include <Wire.h>
#include <Arduino.h>
extern "C" {
#include "util/arduino_mpu9250_i2c.h"
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}
#include <MPU9250_RegisterMap.h>

#define MPU9250_ADDR 0x68
#define com SerialUSB
#define i2c_read arduino_i2c_read
#define i2c_write arduino_i2c_write

// global types & constants
#define SD_LOG_WRITE_BUFFER_SIZE 64
#define BUFFER_SIZE 64
#define MAX_BUFFER_LENGTH 63

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
struct Acceleration{
  /* Raw data */
  short ax;
  short ay;
  short az;
  /* time in ms from MPU9250 */
  unsigned long t;};
Acceleration accelBuffer[BUFFER_SIZE];

unsigned short _aSense;

unsigned long timeStamp, previousTime;
unsigned short dataPointer; // how large is short - 8 bits? only 256 then

void fail(void)
    { 
      com.println("MPU9520 Accelerator not available");
      com.println("Ctrl+Z should finish the program.");
      while(1);
    };
    
void setup() 
{
  com.begin(115200);

// Initialize global vars
  dataPointer = 0;
  previousTime = timeStamp = millis();

  // MPU::Begin
  inv_error_t result;
  struct int_param_s int_param;
  Wire.begin(); //communication between ARM Cortex and MPU9250
  result = mpu_init(&int_param);

  if (result != INV_SUCCESS) fail();

  
  // Init accelerometer
  mpu_set_bypass(1); // Place all slaves (including compass) on primary bus
  mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO); // to keep PLL clock
  mpu_set_sample_rate(500);
  // set and check full scale range
  mpu_set_accel_fsr(16);
  if (int st = mpu_get_accel_sens(&_aSense)) fail();
  // Set interrupt enable when raw data ready
  unsigned char bit1 = 0x1;
  i2c_write(MPU9250_ADDR, MPU9250_INT_ENABLE, 1, &bit1);
  
  delay(2000);

  // Probe the sensor
  unsigned char c;
  mpu_read_reg(MPU9250_WHO_AM_I, &c);
  
  com.print("MPU9520 Who am I returns 0x"); com.println(c, HEX);

  // Check the configuration and print values
  if (c == 0x71) // WHO_AM_I returns 0x71 for MPU9250
  {
    com.println("MPU9250 is online...");

    com.println("Reg (HEX)       Value");
    // Sample rate
    unsigned char registerByte;
    unsigned char microReg = MPU9250_SMPLRT_DIV;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" SMPLRT_DIV ");com.println(registerByte,BIN);
    
    // General configuration
    mpu_read_reg(MPU9250_CONFIG, &registerByte);
    com.print(MPU9250_CONFIG,HEX);com.print("          ");com.println(registerByte,BIN);

    //Accelerometer configuration 1
    microReg = MPU9250_ACCEL_CONFIG;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX); com.print(" FS_SEL     ");
    com.println((registerByte >> ACCEL_CONFIG_ACCEL_FS_SEL)&MPU9250_ACCEL_FS_SEL_MASK,BIN);

    //Digital filter and ?? accel_fchoice_b
    microReg = MPU9250_ACCEL_CONFIG_2;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print("        ");com.println(registerByte,BIN);
  
    // I2C master clock frequency
    microReg = MPU9250_I2C_MST_CTRL;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" I2C MST CTRL ");com.println(registerByte,BIN);
    
    // Interrupt setting
    microReg = MPU9250_INT_ENABLE;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" INT_EN    ");com.println(registerByte,BIN);

    microReg = MPU9250_INT_STATUS;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" INT_ST    ");com.println(registerByte,BIN);
    
    // Power managment 1
    microReg = MPU9250_PWR_MGMT_1;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" PWR_1 1-5 are PLL CLK ");com.println(registerByte&7);

    // Power management 2
    microReg = MPU9250_PWR_MGMT_2;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" PWR_2     ");com.println(registerByte,BIN);

    long aBias[3];
    mpu_read_6500_accel_bias(aBias);
    com.print("Accel offsets: aX "); com.print(aBias[0]);
    com.print(" aY "); com.print(aBias[1]);
    com.print(" aZ "); com.println(aBias[2]);

    com.print(" Accel sensitivity "); com.println(_aSense);
  
  } // MPU9250 check loop

}

void loop() {

  unsigned char intStatusReg;
  mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg);
  
  //unsigned long current = micros();
  if (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT))
  {
    //unsigned int statusReady = (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
    
    // read accel data directly from MPU9250 register
    short data[3];
    timeStamp = millis();
    unsigned char tmp[6];
    if (!i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, tmp))
    {
      accelBuffer[dataPointer].ax = (tmp[0] << 8) | tmp[1];
      accelBuffer[dataPointer].ay = (tmp[2] << 8) | tmp[3];
      accelBuffer[dataPointer].az = (tmp[4] << 8) | tmp[5];
      accelBuffer[dataPointer].t = timeStamp;
      dataPointer++;
    }
  } // INT_STATUS ready
  //com.println(micros()-current);
  
  if (dataPointer == MAX_BUFFER_LENGTH) {
    //buffer full
    // Do not overload the USB port, print out every second
    if ((timeStamp-previousTime) >> 10){
      for (int i=0; i<MAX_BUFFER_LENGTH; i++) {
        com.print(i);com.print(" ");
        com.print(accelBuffer[i].ax/_aSense,DEC); com.print(" "); 
        com.print(accelBuffer[i].ay/_aSense,DEC); com.print(" ");
        com.print(accelBuffer[i].az/_aSense,DEC); com.print(" ");
        com.print(accelBuffer[i].t); com.println(" ms");
      }
      previousTime = timeStamp;
    }
    dataPointer = 0;
  }
  
  /*
  if (measurement.length() > SD_LOG_WRITE_BUFFER_SIZE){
    com.print(measurement);
    measurement="";
  }*/
  
}

