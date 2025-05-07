#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include "myMessage.h"
#define LEN_DATA 4*14
#define LEN_MSG 2+LEN_DATA+2

class Nano33Sensor
{
    private:
        Stream& port;
        MyCRC crc = MyCRC(16, 0x8005, 0xFFFF);
        uint8_t buffer[LEN_MSG];
        uint8_t status = 0;
        bool newData = false;
        my_data_3f gyro, accl, euler, mag;
        my_data_u4 time, bar;

        void parseData();

    public:
        Nano33Sensor(Stream& serial) : port (serial){};
        ~Nano33Sensor();
        bool readData();
        void printByte();
        bool isNewData() { return newData; }
        float getTime() { return time.ulong_val * 1e-6; }
        my_data_3f getGyro() { return gyro; }
        my_data_3f getAccl() { return accl; }
        my_data_3f getMag()  { return mag;  }
        my_data_3f getEuler() { return euler; }
        float getBar() { return bar.float_val;}
 
};
    

#endif