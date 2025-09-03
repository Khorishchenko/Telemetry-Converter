#ifndef MSP_CONVERTER_H
#define MSP_CONVERTER_H

#include <vector>
#include <cstdint>
#include <string>
#include <iostream>

// Прототип для функції обчислення контрольної суми MSP
uint8_t calculateMspChecksum(const std::vector<uint8_t>& data);

// Прототип для функції перетворення MSP-даних в MAVLink-пакети
void convertMspToMavlink(const std::vector<uint8_t>& mspPayload, uint8_t commandCode);

// Прототип для функції виведення буфера у шістнадцятковому форматі
void printHexBuffer(const uint8_t* buffer, uint16_t length);

// Прототип для функції запису в файл
void writeToFile(const std::string& filename, const uint8_t* buffer, uint16_t length);

// Прототип для загальної функції парсингу MSP
void parseMspData(FILE* input);


void sendMavlinkPacketOverUdp(const uint8_t* buffer, uint16_t length, const std::string& ip_address, int port);

#endif // MSP_CONVERTER_H