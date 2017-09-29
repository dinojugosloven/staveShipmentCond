import sys, os, serial, threading

def monitor():

	ser = serial.Serial(COMPORT, BAUDRATE, timeout=0)

	index = 0
	fileName = LOG_FILE_PREFIX + str(index) + ".log"
	textFile = open(fileName, "w")
	buffer = ''
	while (1):
		if(textFile.tell() < MAX_FILE_LENGTH):
			line = ser.readline()
			if (line != ""):
				if (sys.getsizeof(buffer) < MAX_BUFFER_SIZE):
					buffer += line
				else:
					textFile.write(buffer)
					buffer = ''
		else:
			textFile.close()
			index += 1
			fileName = LOG_FILE_PREFIX+str(index)+".log"
			textFile = open(fileName, "w")
	textFile.close()
	print "Stop Monitoring"

""" -------------------------------------------
MAIN APPLICATION
"""  

print "Start Serial Monitor"
print

LOG_FILE_PREFIX = "data_"
MAX_BUFFER_SIZE = 2048
MAX_FILE_LENGTH = 8000000
COMPORT = "COM4"
BAUDRATE = 115200

monitor()