#ifndef MSP_CONVERTER_H
#define MSP_CONVERTER_H

#include <vector>
#include <cstdint>
#include <string>
#include <iostream>

uint8_t calculateMspChecksum(const std::vector<uint8_t>& data);
// void convertMspToMavlinkFile(const std::vector<uint8_t>& mspPayload, uint8_t commandCode);
void convertMspToMavlink(const std::vector<uint8_t>& mspPayload, uint16_t commandCode);
void printHexBuffer(const uint8_t* buffer, uint16_t length);
void writeToFile(const std::string& filename, const uint8_t* buffer, uint16_t length);
void sendMavlinkPacketOverUdp(const uint8_t* buffer, uint16_t length, const std::string& ip_address, int port);

class MspParser {
private:
    enum State {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_M,
        MSP_HEADER_ARROW,
        MSP_HEADER_SIZE,
        MSP_HEADER_CODE,
        MSP_PAYLOAD,
        MSP_CHECKSUM
    } currentState;

    std::vector<uint8_t> payloadBuffer;
    uint8_t dataSize;
    uint8_t checksum;

    // Ось тут додаємо змінні для заголовка та розміру
    uint8_t flags;       // перший байт після "$M>"
    uint8_t cmdLSB;      // function LSB
    uint8_t cmdMSB;      // function MSB
    uint8_t sizeLSB;     // LSB розміру payload
    uint8_t sizeMSB;     // MSB розміру payload
    uint16_t payloadSize; // розмір payload

public:
    MspParser() : currentState(MSP_IDLE), dataSize(0), checksum(0) {}
    void parseData(const char* data, size_t length);
    State getState() const { return currentState; }

    size_t getPayloadSize() const { return dataSize; }
 
    // Пакет вважається завершеним, якщо payload отримано і checksum зійшлась
    bool isPacketComplete() const { return !payloadBuffer.empty() && currentState == MSP_IDLE; }

    // Повертаємо останній обчислений checksum (для дебагу)
    uint8_t getChecksum() const { return checksum; }
};

#endif // MSP_CONVERTER_H