/*
 * Copyright 2023 Ethan. All rights reserved.
 */

#include "serial.h"

bool HardwareSerial::begin(int baudRate, const char *serialPort) {
	if (fd != -1) {
		::close(fd);
		fd = -1;
	}

	if (serialPort == NULL)
		return false;

	fd = ::open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == -1) {
		perror("open:");
		return false;
	}
	fcntl(fd, F_SETFL, FNDELAY);
	tcgetattr(fd, &orgopt);
	tcgetattr(fd, &curopt);
	speed_t CR_BAUDRATE;
	switch (baudRate) {
		case 9600:
			CR_BAUDRATE = B9600;
			break;
		case 19200:
			CR_BAUDRATE = B19200;
			break;
		case 38400:
			CR_BAUDRATE = B38400;
			break;
		case 57600:
			CR_BAUDRATE = B57600;
			break;
		case 115200:
			CR_BAUDRATE = B115200;
			break;
		case 500000:
			CR_BAUDRATE = B500000;
			break;
		case 1000000:
			CR_BAUDRATE = B1000000;
			break;
		default:
			CR_BAUDRATE = B115200;
			break;
	}
	cfsetispeed(&curopt, CR_BAUDRATE);
	cfsetospeed(&curopt, CR_BAUDRATE);

	printf("serial speed %d\n", baudRate);
	// Mostly 8N1
	curopt.c_cflag &= ~PARENB;
	curopt.c_cflag &= ~CSTOPB;
	curopt.c_cflag &= ~CSIZE;
	curopt.c_cflag |= CS8;
	curopt.c_cflag |= CREAD;
	curopt.c_cflag |= CLOCAL; // disable modem statuc check
	cfmakeraw(&curopt);       // make raw mode
	curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	if (tcsetattr(fd, TCSANOW, &curopt) == 0) {
		return true;
	} else {
		perror("tcsetattr:");
		return false;
	}
}

int HardwareSerial::setBaudRate(int baudRate) {
	if (fd == -1) {
		return -1;
	}
	tcgetattr(fd, &orgopt);
	tcgetattr(fd, &curopt);
	speed_t CR_BAUDRATE;
	switch (baudRate) {
		case 9600:
			CR_BAUDRATE = B9600;
			break;
		case 19200:
			CR_BAUDRATE = B19200;
			break;
		case 38400:
			CR_BAUDRATE = B38400;
			break;
		case 57600:
			CR_BAUDRATE = B57600;
			break;
		case 115200:
			CR_BAUDRATE = B115200;
			break;
		case 230400:
			CR_BAUDRATE = B230400;
			break;
		case 500000:
			CR_BAUDRATE = B500000;
			break;
		default:
			break;
	}
	cfsetispeed(&curopt, CR_BAUDRATE);
	cfsetospeed(&curopt, CR_BAUDRATE);
	return 1;
}

int HardwareSerial::read() {
	unsigned char dat = 0;
	if (fd == -1) {
		return -1;
	}

	if (::read(fd, &dat, 1) <= 0) {
		return -1;
	}

	return dat;
}

int HardwareSerial::write(unsigned char *nDat, int nLen) {
	if (fd == -1) {
		return -1;
	}
	return ::write(fd, nDat, nLen);
}

void HardwareSerial::end() {
	::close(fd);
	fd = -1;
}
