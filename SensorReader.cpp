#include "SensorReader.h"


Nano33Sensor::~Nano33Sensor() {}

void Nano33Sensor::printByte() {
    while (port.available() > 0) {
        Serial.print(port.read());
        Serial.print(" ");
    }
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
    memcpy(&temp, &buffer[58], sizeof(my_data_u4));
}


