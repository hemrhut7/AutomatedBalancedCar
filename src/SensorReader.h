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


// 封包解析狀態
enum ParseState {
  WAIT_HEADER_1,  // 等待第一個header (0x7B)
  WAIT_HEADER_2,  // 等待第二個header (0x00)
  READ_TYPE,      // 讀取類型
  READ_PAYLOAD_TYPE0, // 讀取type=0的payload
  READ_CHECKSUM,  // 讀取checksum (type=0)
  READ_TRAILER,   // 讀取trailer
  READ_PAYLOAD_TYPE1  // 讀取type=1的字串
};

// 解析結果結構體
struct ParseResult {
  bool success;         // 是否成功解析
  uint8_t type;         // 封包類型 (0 or 1)
  int16_t data[3];      // type=0的數據 (三個int16_t)
  String string_data;   // type=1的字串數據
};

ParseResult readSerialPacket(Stream &port);
void sendBTMessage(Stream &port, float a, float target_a, float b, float target_b, float c, float target_c);
inline void writeFloat(float value, uint8_t* buffer);

#endif