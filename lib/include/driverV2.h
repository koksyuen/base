#include "serialport.h"

#include <vector>
#include <string>
#include <functional>

#ifndef _DriverV2
#define _DriverV2

using namespace std;

namespace library
{

class Encoder
{
public:
  int left;
  int right;
};

class IMU
{
public:
  float x;
  float y;
  float z;
};

class Driver2Sensor
{
public:
  Encoder encoder;
  IMU accelerometer;
  IMU gyroscope;
  IMU magnetometer;
  bool valid;
};

class DriverV2
{
public:
  DriverV2(string);
  DriverV2(DriverV2 &&) = default;
  ~DriverV2();

  void start();
  void keepAlive();
  void UpdateControls(double left, double right);

  function<void(Driver2Sensor)> onData;

private:
  SerialPort port;

  enum status
  {
    IDLE,
    LENGTH,
    RECEIVING,
    END
  };
  status receiveStatus;

  int length;
  vector<unsigned char> received;

  vector<unsigned char> createFrame(unsigned char, vector<unsigned char>);
  void serialportReceived(vector<unsigned char>);
  float floatConvertor(uint32_t);
  uint32_t uint32Convertor(unsigned char *);
  Driver2Sensor checkFrame();
};

} // namespace library

#endif