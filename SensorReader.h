#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include "myMessage.h"
#define LEN_DATA 4 * 14             // time (4), omg (12), acc (12), mag (12), ori (12), bar (4) = 56 bytes
#define LEN_MSG 2 + LEN_DATA + 2    // header (2) + data (56) + CRC-16 (2) = 60 bytes

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
        void printBuffer();
        bool isNewData() { return newData; }
        float getTime() { return time.ulong_val * 1e-3; }
        my_data_3f getGyro() { return gyro; }
        my_data_3f getAccl() { return accl; }
        my_data_3f getMag()  { return mag;  }
        my_data_3f getEuler() { return euler; }
        float getBar() { return bar.float_val;}
 
};


bool parseCommand(Stream& port, uint8_t *buffer);
void sendBTMessage(Stream &port, float a, float target_a, float b, float target_b, float c, float target_c);
// uint8_t writeFloat2Buffer(float value, uint8_t *buffer);
inline void writeFloat(float value, uint8_t* buffer);

#endif