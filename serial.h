/*
 * Copyright 2023 Ethan. All rights reserved.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

class HardwareSerial {
public:
	HardwareSerial(){fd = -1;}
	~HardwareSerial(){}

	int setBaudRate(int baudRate);
	bool begin(int baudRate, const char *serialPort);
	void end();
	int read();
	int write(unsigned char *nDat, int nLen);

private:
	int fd;                // serial port handle
	struct termios orgopt; // fd ort opt
	struct termios curopt; // fd cur opt
};

#endif /* __SERIAL_H__*/
