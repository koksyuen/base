#include "driverV2.h"
#include <iostream>
#include <functional>
#include <memory.h>
#include <math.h>

namespace library
{

DriverV2::DriverV2(string portname)
{
    port = SerialPort(portname);
    port.onReceived = bind(&DriverV2::serialportReceived, this, placeholders::_1);
    port.connect();
    receiveStatus = IDLE;
}

DriverV2::~DriverV2()
{
    port.disconnect();
}

void DriverV2::start()
{
    port.send(createFrame(1, {}));
}

void DriverV2::keepAlive()
{
    port.send(createFrame(0, {}));
}

void DriverV2::UpdateControls(double left, double right)
{
    vector<unsigned char> data;
    int _left = (int)round(left * 4095);
    int _right = (int)round(right * 4095);

    data.push_back(_left >> 8);
    data.push_back(_left);
    data.push_back(_right >> 8);
    data.push_back(_right);

    port.send(createFrame(2, data));
}

void DriverV2::serialportReceived(vector<unsigned char> data)
{
    for (unsigned char d : data)
    {
        switch (receiveStatus)
        {
        case IDLE:
            if (d == 0xF5)
            {
                receiveStatus = LENGTH;
                received.clear();
                received.push_back(d);
            }
            break;
        case LENGTH:
            received.push_back(d);
            length = d;
            receiveStatus = RECEIVING;
            break;
        case RECEIVING:
            received.push_back(d);
            if (--length == 0)
            {
                receiveStatus = END;
            }
            break;
        case END:
            receiveStatus = IDLE;
            received.push_back(d);
            if (d == 0xF6)
            {
                if (onData != NULL)
                {
                    onData(checkFrame());
                }
            }
            break;
        }
    }
}

float DriverV2::floatConvertor(uint32_t dword)
{
    float f;
    memcpy(&f, &dword, 4);
    return f;
}

uint32_t DriverV2::uint32Convertor(unsigned char *data)
{
    return *data << 24 | (*(data + 1)) << 16 | (*(data + 2)) << 8 | (*(data + 3));
}

Driver2Sensor DriverV2::checkFrame()
{
    Driver2Sensor sensor;
    int sum = 0;
    if (received.size() > 4)
    {
        for (int i = 2; i < received.size() - 2; i++)
        {
            sum ^= received[i];
        }
        if (received[received.size() - 2] == sum)
        {
            sensor.encoder.left = int16_t(received[7] << 8 | received[8]) + ((double)int16_t(received[9] << 8 | received[10])) / 2000;
            sensor.encoder.right = int16_t(received[11] << 8 | received[12]) + ((double)int16_t(received[13] << 8 | received[14])) / 2000;

            sensor.accelerometer.x = floatConvertor(uint32Convertor(&received[15])) * 9.80665;
            sensor.accelerometer.y = floatConvertor(uint32Convertor(&received[19])) * 9.80665;
            sensor.accelerometer.z = floatConvertor(uint32Convertor(&received[23])) * 9.80665;

            sensor.gyroscope.x = floatConvertor(uint32Convertor(&received[27])) * (M_PI / 180);
            sensor.gyroscope.y = floatConvertor(uint32Convertor(&received[31])) * (M_PI / 180);
            sensor.gyroscope.z = floatConvertor(uint32Convertor(&received[35])) * (M_PI / 180);

            sensor.magnetometer.x = floatConvertor(uint32Convertor(&received[39]));
            sensor.magnetometer.y = floatConvertor(uint32Convertor(&received[43]));
            sensor.magnetometer.z = floatConvertor(uint32Convertor(&received[47]));

            sensor.valid = true;
        }
        else
        {
            sensor.valid = false;
        }
    }
    else
    {
        sensor.valid = false;
    }
    return sensor;
}

vector<unsigned char> DriverV2::createFrame(unsigned char code, vector<unsigned char> data)
{
    vector<unsigned char> frame;
    frame.push_back(0xF5);
    frame.push_back(data.size() + 2);
    frame.push_back(code);
    int sum = code;

    for (unsigned int value : data)
    {
        sum ^= value;
        frame.push_back(value);
    }
    frame.push_back(sum);
    frame.push_back(0xF6);
    return frame;
}
} // namespace library