// Default logging parameters
#define ENABLE_TIME_LOG       true
#define ENABLE_CALCULATED_LOG true

////////////////////////
// Serial Port Config //
////////////////////////
#define ENABLE_UART_LOGGING true
// Select the Serial port to log to. Either SERIAL_PORT_USBVIRTUAL
// or LOG_PORT SERIAL_PORT_HARDWARE (SerialUSB or Serial1)
#define LOG_PORT SERIAL_PORT_USBVIRTUAL
#define SERIAL_BAUD_RATE 115200 // Serial port baud

///////////////////////
// SD Logging Config //
///////////////////////
#define ENABLE_SD_LOGGING false // Default SD logging (can be changed via serial menu)
#define LOG_FILE_INDEX_MAX 999 // Max number of "logXXX.txt" files
#define LOG_FILE_PREFIX "log"  // Prefix name for log files
#define LOG_FILE_SUFFIX "txt"  // Suffix name for log files
#define SD_MAX_FILE_SIZE 8000000 // 8MB max file size, increment to next file before surpassing
#define SD_LOG_WRITE_BUFFER_SIZE 2048 // Experimentally tested to produce 100Hz logs

////////////////
// LED Config //
////////////////
#define HW_LED_PIN 13        // LED attached to pin 13
#define UART_BLINK_RATE 1000 // Blink rate when only UART logging

/////////////////////
// Serial Commands //
/////////////////////
#define PAUSE_LOGGING     ' ' // Space - Pause SD/UART logging
#define ENABLE_TIME       't' // Enable/disable time log (milliseconds)
#define SET_LOG_RATE      'r' // Adjust logging rate from 1-200 Hz (in 10 Hz increments)

//////////////////////////
// Hardware Definitions //
//////////////////////////
// Danger - don't change unless using a different platform
#define MPU9250_INT_PIN 4
#define SD_CHIP_SELECT_PIN 38
#define MPU9250_INT_ACTIVE LOW
