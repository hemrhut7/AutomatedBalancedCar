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




// buffer must have 11 bytes length
bool parseCommand(Stream &port, uint8_t *buffer){
    READING_DATA_STATE state = EXPECTING_HEADER;
    const uint8_t header[3] = {0x7B, 0, 0};
    const uint8_t trailer = 0x7D;

    const uint8_t header_size = 3;
    const uint8_t len_data = 6;
    const uint8_t len_msg = 11;
    uint8_t bytes_received = 0;

    while (port.available() && bytes_received < len_msg){
        uint8_t data = port.read();

        switch (state) {
        case EXPECTING_HEADER:
            if (data != header[bytes_received]){ 
                bytes_received = 0; 
            }


            buffer[bytes_received++] = data;
            if (bytes_received >= header_size){
                state = EXPECTING_PAYLOAD;
            }
            break;

        case EXPECTING_PAYLOAD:
            buffer[bytes_received++] = data;
            if (bytes_received >= header_size + len_data){
                state = EXPECTING_CHECKSUM;
            }
            break;
        
        case EXPECTING_CHECKSUM:
            if (!xor_checksum(buffer, header_size + len_data, data)){
                return false;
            }
            buffer[bytes_received++] = data;
            state = EXPECTING_TRAILER;
            break;
        
        case EXPECTING_TRAILER:
            if (data != trailer) {return false;}
            buffer[bytes_received++] = data;
            break;

        default:
            return false;
        }
    }

    return bytes_received == len_msg;
}

void sendBTMessage(Stream &port, float a, float target_a, float b, float target_b, float c, float target_c){
    uint8_t buffer[23] = {0x7B, 0x00};
    uint8_t *ptr = buffer + 2;

    // 將浮點數轉為整數並寫入大端格式
    writeFloat(a, ptr); ptr += 2;
    writeFloat(target_a, ptr); ptr += 2;
    *ptr++ = 0; *ptr++ = 0; // 直接寫入0

    writeFloat(b, ptr); ptr += 2;
    writeFloat(target_b, ptr); ptr += 2;
    *ptr++ = 0; *ptr++ = 0;

    writeFloat(c, ptr); ptr += 2;
    writeFloat(target_c, ptr); ptr += 2;
    *ptr++ = 0; *ptr++ = 0;

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
    int16_t int_value = (int16_t)(value * 1000.0f);
    buffer[0] = (int_value >> 8) & 0xFF; // MSB
    buffer[1] = int_value & 0xFF;       // LSB
}