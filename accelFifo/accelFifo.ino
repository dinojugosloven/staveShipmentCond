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

#include "config.h"

#define MPU9250_ADDR 0x68
#define com SerialUSB
#define timer micros
#define i2c_read arduino_i2c_read
#define i2c_write arduino_i2c_write

// global types & constants
#define SAMPLING_RATE 1

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
long aBias[3];

unsigned long timeStamp, previousTime; // 64 bit
unsigned long timeReadFifo;
const unsigned long samplingPeriod = 1000.0/SAMPLING_RATE; // in ms 
unsigned short dataPointer; // how large is short - 8 bits? only 256 then

void fail(void)
    { 
      com.println("MPU9520 Accelerator not available");
      com.println("Ctrl+Z should finish the program.");
      while(1);
    };

void fifoError(int _fifoStatus)
{
  com.print("FIFO Error "); com.println(_fifoStatus);
  if (_fifoStatus == FIFO_OVERFLOW) mpu_reset_fifo();
}
    
void setup() 
{
  com.begin(115200);

// Initialize global vars
  dataPointer = 0;
  previousTime = timeStamp = timer();

  // MPU::Begin
  inv_error_t result;
  struct int_param_s int_param;
  Wire.begin(); //communication between ARM Cortex and MPU9250
  result = mpu_init(&int_param);

  if (result != INV_SUCCESS) fail();

  
  // Init accelerometer
  mpu_set_bypass(1); // Place all slaves (including compass) on primary bus
  mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO); // to keep PLL clock
  mpu_set_sample_rate(SAMPLING_RATE);
  // set and check full scale range
  mpu_set_accel_fsr(16);
  if (int st = mpu_get_accel_sens(&_aSense)) fail();
  // Set interrupt enable when raw data ready
  unsigned char bit1 = 0x1;
  i2c_write(MPU9250_ADDR, MPU9250_INT_ENABLE, 1, &bit1);
  // reset fifo, enable overflow bit and configure to write accel data
  mpu_configure_fifo(INV_XYZ_ACCEL);
  //unsigned char tempBits;
  //mpu_read_reg(MPU9250_INT_ENABLE, &tempBits);
  //tempBits = tempBits | INT_ENABLE_FIFO_OVERFLOW_EN;
  //i2c_write(MPU9250_ADDR, MPU9250_INT_ENABLE, 1, &tempBits);

  
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

  // FIFO configuration
    microReg = MPU9250_FIFO_EN;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" FIFO_EN?  ");com.println(registerByte>>3 & true,BIN);
  
    // I2C master clock frequency
    microReg = MPU9250_I2C_MST_CTRL;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" I2C MST CTRL ");com.println(registerByte,BIN);
    
    // Interrupt setting
    microReg = MPU9250_INT_PIN_CFG;
    mpu_read_reg(microReg, &registerByte);
    com.print(microReg,HEX);com.print(" INT_CFG    ");com.println(registerByte,BIN);
    
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


    mpu_read_6500_accel_bias(aBias);
    com.print("Accel offsets: aX "); com.print(aBias[0]);
    com.print(" aY "); com.print(aBias[1]);
    com.print(" aZ "); com.println(aBias[2]);

    com.print("Accel sensitivity "); com.println(_aSense);
    com.print("Sampling period [ms] "); com.println(samplingPeriod); 

    com.println(" ************************************************************** ");
  
  } // MPU9250 check loop

}
unsigned int counter=0;
  unsigned char remaining=1;
void loop() {

  // check if fifo overflowed
  unsigned char tmp[6];
  if (i2c_read(MPU9250_ADDR, MPU9250_INT_STATUS, 1, tmp)) fifoError(READ_ERROR);
  if (tmp[0] >> INT_STATUS_FIFO_OVERFLOW_INT & true) fifoError(FIFO_OVERFLOW);
  
  if(fifoCounter() >= 12) // 2 readings of accel, 10*6 bytes 
  {
    //check if it is now data; this clears the interrupt
    timeReadFifo = timer();
    //com.println(timeReadFifo);
    //check that new data is available
    mpu_read_reg(MPU9250_INT_STATUS, tmp);
    if (tmp[0] & true) {
      do {
        int fifoStatus=mpu_read_fifo_stream(6, tmp, &remaining);
        com.print("Remaining "); com.println(remaining);
        if (!remaining) break;
        if (fifoStatus == INV_SUCCESS) 
        {
        accelBuffer[dataPointer].ax = (tmp[0] << 8) | tmp[1];
        accelBuffer[dataPointer].ay = (tmp[2] << 8) | tmp[3];
        accelBuffer[dataPointer].az = (tmp[4] << 8) | tmp[5];
        accelBuffer[dataPointer].t = timeStamp;
        dataPointer++;
        }
        else {
          fifoError(fifoStatus);
          com.println(counter);
          counter++;
        }
      } while (remaining>0);
    }
    
    timeStamp = timer();
    //com.println(timeStamp);
    timeReadFifo = timeStamp - timeReadFifo;

  } // FIFO available
  
  if (dataPointer >= MAX_BUFFER_LENGTH) {
    //buffer full
    // Do not overload the USB port, print out every second
    //if ((timeStamp-previousTime)>>20){
      for (int i=0; i<MAX_BUFFER_LENGTH; i++) {
        com.print(i);com.print(" ");
        com.print((accelBuffer[i].ax),DEC); com.print(" "); 
        com.print((accelBuffer[i].ay),DEC); com.print(" ");
        com.print((accelBuffer[i].az),DEC); com.print(" ");
        com.print(accelBuffer[i].t); com.println(" us");
        com.print("Time now ");com.println(timeStamp);
        com.print("Reading of FIFO took ");com.println(timeReadFifo);
      }
      previousTime = timeStamp;
    //}
    dataPointer = 0;
  }
  

}

int fifoCounter(){
  unsigned char fifoH, fifoL;
  
  if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
    return 0;
  if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
    return 0;
  
  return (fifoH << 8 ) | fifoL;
}

