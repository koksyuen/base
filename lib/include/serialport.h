#include <termios.h>

#include <thread>
#include <vector>
#include <functional>

#ifndef _SerialPort
#define _SerialPort

using namespace std;

namespace library
{
class SerialPort
{
public:
  SerialPort();
  SerialPort(string);

  void send(vector<unsigned char>);

  void connect();
  void disconnect();

  function<void(vector<unsigned char>)> onReceived;

private:
  string _portname;
  struct termios tio;
  struct termios stdio;

  int tty_fd;
  fd_set rdset;
  bool running;

  void process();

  thread t;
};
} // namespace library

#endif