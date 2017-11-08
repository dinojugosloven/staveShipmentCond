/** Dino Tahirovic, QMUL
 *  30/10/2017
 */
#include <SD.h>
#include "config.h"
 
#include <Wire.h>
#include <Arduino.h>
extern "C" {
#include "util/arduino_mpu9250_i2c.h"
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}
#include <MPU9250_RegisterMap.h>


// Rename some typse
#define com SerialUSB
#define timer micros
#define i2c_read arduino_i2c_read
#define i2c_write arduino_i2c_write
typedef int inv_error_t;

enum t_axisOrder {
  X_AXIS, // 0
  Y_AXIS, // 1
  Z_AXIS  // 2
};

// SOME CONSTANTS
#define SAMPLING_RATE 10

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
//char accelBuffer[BUFFER_SIZE];

unsigned short _aSense;

unsigned long timeStamp, previousTime; // in micros
unsigned long  hourScaler; // in milliseconds
unsigned short dataPointer; // how large is short - 8 bits? only 256 then

File logFile;

/////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; // Keeps track of if SD card is plugged in
String logFileName; // Active logging file
String logFileBuffer; // Buffer for logged data. Max is set in config

void fail(void)
    {
      // Blue led ON --> error
      digitalWrite(HW_LED_PIN, true); 
      com.println("MPU9520 Accelerator not available");
      com.println("Ctrl+Z should finish the program.");
      while(1);
    };

void setup() {

  com.begin(115200);

  // Blue led goes off if everything is OK
  digitalWrite(HW_LED_PIN, false);

// Initialize global vars
  dataPointer = 0;
  previousTime = timeStamp = timer();
  hourOffset = 0;

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
  
  delay(2000);

  // Probe the sensor
  unsigned char c;
  mpu_read_reg(MPU9250_WHO_AM_I, &c);
  
  com.print("MPU9520 Who am I returns 0x"); com.println(c, HEX);

  // Check the configuration and print values
  if (c == 0x71) printConfiguration();// WHO_AM_I returns 0x71 for MPU9250
  

  // Check for the presence of an SD card, and initialize it:
  if ( initSD() )
  {
    sdCardPresent = true;
    // Get the next, available log file name
    logFileName = nextLogFile(); 
    
  }
    // Open the current file name:
  logFile = SD.open(logFileName, FILE_WRITE);
}

void loop() {

  //check for interrupt signal
  unsigned char intStatusReg;
  mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg);
  
  //unsigned long current = micros();
  if (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT))
  {
    //unsigned int statusReady = (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
    
    // read accel data directly from MPU9250 register
    timeStamp = timer();
    unsigned char tmp[6];
    if (!i2c_read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, tmp))
    {
      // sprintf is too long, 800 us because of formating
      //int writeStatus = sprintf(accelBuffer, "%lu, %hd, %hd, %hd \n", 
        
        accelBuffer.ax = (tmp[0] << 8) | tmp[1];
        accelBuffer.ay = (tmp[2] << 8) | tmp[3];
        accelBuffer.az = (tmp[4] << 8) | tmp[5]);
        accelBuffer.t = hourScaler + timeStamp>>10, 

        // If the file will get too big with this new string, create
        // a new one, and open it.
        if (logFile.size() > (SD_MAX_FILE_SIZE - MAX_BUFFER_LENGTH))
        {
          logFile.close();
          logFileName = nextLogFile();
          logFile = SD.open(logFileName, FILE_WRITE);
        }
        logFile.println(accelBuffer);
        logFile.flush();
    };
    
    com.print("Read Loop time "); com.println((timer()-timeStamp));
    String serialOutput = String(accelBuffer.t) + String(accelBuffer.ax);
    com.println(serialOutput);
    /*{
      accelBuffer[dataPointer].ax = (tmp[0] << 8) | tmp[1];
      accelBuffer[dataPointer].ay = (tmp[2] << 8) | tmp[3];
      accelBuffer[dataPointer].az = (tmp[4] << 8) | tmp[5];
      accelBuffer[dataPointer].t = timeStamp;
      dataPointer++;
    }*/
  } // INT_STATUS ready
  else if{
    // check the status of timer. If it is larger then
    // 1 hour = 3,600,000,000 us
    // add hour millisecond offset and reset the timer
    if (timeStamp > HourInMicros) {
      hourScaler += 3600000;
      timeStamp = 0;
    }
  }
  

  //com.print("Read/Write Loop time "); com.println(timer()-timeStamp);
}

bool initSD(void)
{
  // SD.begin should return true if a valid SD card is present
  if ( !SD.begin(SD_CHIP_SELECT_PIN) )
  {
    com.println("SD card not present.");
    digitalWrite(HW_LED_PIN, true);
    return false;
  }
  
  com.println("SD card present.");
  return true;
}

// Log a string to the SD card
bool sdLogString(Acceleration* toLog)
{
  // Open the current file name:
  File logFile = SD.open(logFileName, FILE_WRITE);

  // If the file will get too big with this new string, create
  // a new one, and open it.
  if (logFile.size() > (SD_MAX_FILE_SIZE - MAX_BUFFER_LENGTH))
  {
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }


  // If the log file opened properly, add the string to it.
  if (logFile)
  {
    for (int i=0; i<MAX_BUFFER_LENGTH; i++) {
      logFile.print(i);logFile.print(" ");
      logFile.print(toLog[i].t); logFile.print(", ");
      logFile.print(toLog[i].ax/_aSense,DEC); logFile.print(", "); 
      logFile.print(toLog[i].ay/_aSense,DEC); logFile.print(", ");
      logFile.println(toLog[i].az/_aSense,DEC);
    }

    logFile.close();

    return true; // Return success
  }

  return false; // Return fail
}

// Find the next available log file. Or return a null string
// if we've reached the maximum file limit.
String nextLogFile(void)
{
  String filename;
  int logIndex = 0;

  // FAT16 limit: file name must be 8.3 characters or less
  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++)
  {
    // Construct a file with PREFIX[Index].SUFFIX
    filename = String(LOG_FILE_PREFIX);
    filename += String(logIndex);
    filename += ".";
    filename += String(LOG_FILE_SUFFIX);
    // If the file name doesn't exist, return it
    if (!SD.exists(filename))
    {
      return filename;
    }
    // Otherwise increment the index, and try again
    logIndex++;
  }

  return "";
}

void printConfiguration(void)
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

  return;
} // MPU9250 check configuration

