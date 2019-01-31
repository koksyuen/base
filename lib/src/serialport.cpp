#include "serialport.h"
#include <iostream>
#include <string>
#include <string.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

namespace library
{
SerialPort::SerialPort()
{
	_portname = "/dev/ttyUSB0";
}
SerialPort::SerialPort(string portname)
{
	_portname = portname;
}

void SerialPort::process()
{
	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;

	tty_fd = open(_portname.c_str(), O_RDWR | O_NONBLOCK); // O_NONBLOCK might override VMIN and VTIME, so read() may return immediately.
	cfsetospeed(&tio, B115200);							   // 115200 baud
	cfsetispeed(&tio, B115200);							   // 115200 baud

	tcsetattr(tty_fd, TCSANOW, &tio);

	// unsigned char test[1024] = {0xF5, 2, 1, 1, 0xF6};
	// vector<unsigned char> test2 = {0xF5, 2, 1, 1, 0xF6};
	unsigned char received[1024];
	// send(test2);
	// write(tty_fd, &test, 5);
	while (running)
	{
		int count = read(tty_fd, &received, 1024);
		if (count > 0)
		{
			vector<unsigned char> data;
			for (int i = 0; i < count; i++)
			{
				data.push_back(received[i]);
			}
			if (onReceived != NULL)
			{
				onReceived(data);
			}
		}
		sleep(0.001);
	}
	close(tty_fd);
}

void SerialPort::send(vector<unsigned char> data)
{
	if (running)
	{
		write(tty_fd, data.data(), data.size());
	}
}

void SerialPort::connect()
{
	if (!running)
	{
		running = true;
		t = thread(&SerialPort::process, this);
	}
}

void SerialPort::disconnect()
{
	if (running)
	{
		running = false;
		t.join();
	}
}

} // namespace library
