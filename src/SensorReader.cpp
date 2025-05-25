#include "SensorReader.h"


Nano33Sensor::~Nano33Sensor() {}

void Nano33Sensor::printByte() {
    while (port.available() > 0) {
        Serial.print(port.read());
        Serial.print(" ");
    }
}

void Nano33Sensor::printBuffer(){
    Serial.write(buffer, LEN_MSG);
}

bool Nano33Sensor::readData(){
    static byte bytes_received = 0;
    newData = false;

    static enum{
        EXPECTING_HEADER,
        EXPECTING_PAYLOAD,
        EXPECTING_CHECKSUM
    } state = EXPECTING_HEADER;


    while (port.available() > 0 && newData==false) {
        byte data = port.read();

        switch (state)
        {
        case EXPECTING_HEADER:{
            if (data != HEADER[bytes_received++]) {
                state = EXPECTING_HEADER;
                bytes_received = 0;
            }
            if (bytes_received >= HEADER_SIZE) {
                memcpy(buffer, HEADER, HEADER_SIZE);
                state = EXPECTING_PAYLOAD;                
            }
            break;
        }
        
        case EXPECTING_PAYLOAD:{
            buffer[bytes_received++] = data;
            if (bytes_received >= HEADER_SIZE + LEN_DATA) {
                state = EXPECTING_CHECKSUM;
            }
            break;
        }
        case EXPECTING_CHECKSUM:{
            buffer[bytes_received++] = data;
            if (bytes_received >= LEN_MSG) {
                if (crc.isCRCPass(buffer, LEN_MSG)) {
                    newData = true; 
                    parseData();
                }
                state = EXPECTING_HEADER;
                bytes_received = 0;
            }
            break;
        }
        default: break;
        }
    }

    return newData;
}

void Nano33Sensor::parseData() {
    memcpy(&time, &buffer[2], sizeof(my_data_u4));
    memcpy(&gyro, &buffer[6], sizeof(my_data_3f));
    memcpy(&accl, &buffer[18], sizeof(my_data_3f));
    memcpy(&mag, &buffer[30], sizeof(my_data_3f));
    memcpy(&euler, &buffer[42], sizeof(my_data_3f));
    memcpy(&bar, &buffer[54], sizeof(my_data_u4));
}



// Serial讀取函數
ParseResult readSerialPacket(Stream &port) {
  static ParseState state = WAIT_HEADER_1; // 靜態狀態
  static uint8_t buffer[9];                // 儲存type=0的header+type+payload+checksum
  static uint8_t index = 0;                // 緩衝區索引
  static String string_buffer = "";        // 儲存type=1的字串
  static uint8_t expected_checksum = 0;    // type=0的預期checksum

  ParseResult result = {false, 0, {0, 0, 0}, ""};

  // 處理所有當前可用的Serial數據
  while (port.available()) {
    uint8_t byte = port.read();
    // Serial.print(byte, HEX); Serial.print(' ');

    switch (state) {
      case WAIT_HEADER_1:
        if (byte == 0x7B) {
          buffer[0] = byte;
          state = WAIT_HEADER_2;
        } // 無效header，保持等待
        break;

      case WAIT_HEADER_2:
        if (byte == 0x00) {
          buffer[1] = byte;
          state = READ_TYPE;
        } else {
          state = WAIT_HEADER_1; // header錯誤，重置
        }
        break;

      case READ_TYPE:
        if (byte == 0 || byte == 1) {
          buffer[2] = byte;
          result.type = byte;
          index = 3; // 準備儲存payload
          if (byte == 0) {
            state = READ_PAYLOAD_TYPE0;
          } else {
            string_buffer = ""; // 清空字串緩衝區
            state = READ_PAYLOAD_TYPE1;
          }
        } else {
          state = WAIT_HEADER_1; // 無效type，重置
        }
        break;

      case READ_PAYLOAD_TYPE0:
        buffer[index++] = byte;
        if (index == 9) { // 讀完6 bytes payload (index 3到8)
          state = READ_CHECKSUM;
        }
        break;

      case READ_CHECKSUM:
        expected_checksum = cal_xor_checksum(buffer, 9); // 計算header+type+payload的checksum
        if (byte == expected_checksum) {
          state = READ_TRAILER;
        } else {
          state = WAIT_HEADER_1; // checksum錯誤，重置
        }
        break;

      case READ_TRAILER:
        if (byte == 0x7D) {
          // 成功解析type=0
          result.success = true;
          // 將buffer[3-8]轉為三個int16_t (大端格式)
          result.data[0] = (int16_t)((buffer[3] << 8) | buffer[4]);
          result.data[1] = (int16_t)((buffer[5] << 8) | buffer[6]);
          result.data[2] = (int16_t)((buffer[7] << 8) | buffer[8]);
          state = WAIT_HEADER_1; // 重置狀態
        } else {
          state = WAIT_HEADER_1; // trailer錯誤，重置
        }
        break;

      case READ_PAYLOAD_TYPE1:
        if (byte == 0x7D) {
          // 成功解析type=1
          result.success = true;
          result.string_data = string_buffer;
          state = WAIT_HEADER_1; // 重置狀態
        } else {
          string_buffer += (char)byte; // 累積字串
        }
        break;
    }
  }

  return result;
}

void sendBTMessage(Stream &port, float a, float target_a, float b, float target_b, float c, float target_c){
    uint8_t buffer[23] = {0x7B, 0x00};
    uint8_t *ptr = buffer + 2;

    // 將浮點數轉為整數並寫入大端格式
    writeFloat(a, ptr); ptr += 2;
    writeFloat(target_a, ptr); ptr += 2;
    *ptr++ = NAN; *ptr++ = NAN; // 直接寫入0

    writeFloat(b, ptr); ptr += 2;
    writeFloat(target_b, ptr); ptr += 2;
    *ptr++ = NAN; *ptr++ = NAN;

    writeFloat(c, ptr); ptr += 2;
    writeFloat(target_c, ptr); ptr += 2;
    *ptr++ = NAN; *ptr++ = NAN;

    writeFloat(12.0f, ptr); ptr += 2;
    buffer[22] = cal_xor_checksum(buffer, 22);
    port.write(buffer, 23);

    // uint8_t index = 2
    // index += writeFloat2Buffer(a, buffer+index);
    // index += writeFloat2Buffer(target_a, buffer+index);
    // index += writeFloat2Buffer(0, buffer+index);

    // index += writeFloat2Buffer(b, buffer+index);
    // index += writeFloat2Buffer(target_b, buffer+index);
    // index += writeFloat2Buffer(0, buffer+index);

    // index += writeFloat2Buffer(c, buffer+index);
    // index += writeFloat2Buffer(target_c, buffer+index);
    // index += writeFloat2Buffer(0, buffer+index);

    // index += writeFloat2Buffer(12.0f, buffer+index);
    // *(buffer + index) = cal_xor_checksum(buffer, 22);
    
}

// uint8_t writeFloat2Buffer(float value, uint8_t *buffer){
//     my_data_u2 int_value;
//     int_value.short_val = value * 1e3;

//     //to small-endian
//     // memcpy(buffer, int_value.bin_val, 2);

//     // to big-endian
//     buffer[1] = (int_value.ushort_val >> 0) & 0xFF;  // LSB
//     buffer[0] = (int_value.ushort_val >> 8) & 0xFF;  // MSB
//     return 2;
// }

// Inline function for float conversion
inline void writeFloat(float value, uint8_t* buffer) {
    int16_t int_value = (int16_t)(value * 100.0f);
    buffer[0] = (int_value >> 8) & 0xFF; // MSB
    buffer[1] = int_value & 0xFF;       // LSB
}