My version of data acquistion software for SparkFun Razor SEN-14001. 
Based on an example by SparkFun, from their library examples.

My version of code is in accelSDcard. The sampling rate is 1000 Hz. I save the raw data as 10 bytes into a binary file on SD card. Due to write/flush of SD card, there is 50 ms gap every 256 readings (my choice of buffer size).

To convert binary to human readable format, one can use readData.cpp
