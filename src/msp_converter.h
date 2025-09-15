#pragma once

#include <vector>
#include <cstdint>
#include <string>


// функції для роботи з MSP і MAVLink
void sendMspV2Request(int fd, uint16_t function);
uint8_t calculateMspChecksum(const std::vector<uint8_t>& data);
uint8_t crc8_dvb_s2(const uint8_t* data, size_t len);
void convertMspToMavlink(const std::vector<uint8_t>& mspPayload, uint16_t commandCode);
void printHexBuffer(const uint8_t* buffer, uint16_t length);
void writeToFile(const std::string& filename, const uint8_t* buffer, uint16_t length);
void sendMavlinkPacketOverUdp(const uint8_t* buffer, uint16_t length, const std::string& ip_address, int port);

// Конфігурація телеметрії (читання з env)
struct TelemetryConfig {
    uint8_t sysid;
    uint8_t compid;
    std::string udp_ip;
    int udp_port;
};

const TelemetryConfig& getTelemetryConfig();


// клас для парсингу MSP-пакетів
class MspParser {
private:
    enum State {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_M,
        MSP_PAYLOAD
    } currentState = MSP_IDLE;

    std::vector<uint8_t> payloadBuffer;
    std::vector<uint8_t> headerBuffer;

public:
    MspParser() = default;
    void parseData(const char* data, size_t length);
};