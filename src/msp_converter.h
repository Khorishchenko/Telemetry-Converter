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




// Станова машина для парсингу MSP
class MspParser
{
private:
    enum State
    {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_M,
        MSP_HEADER_ARROW,
        MSP_HEADER_V1,
        MSP_PAYLOAD
    } currentState = MSP_IDLE;

    std::vector<uint8_t> payloadBuffer;
    uint16_t sizeToRead = 0;

public:
    // Основна функція для обробки вхідних даних
    void parseData(const char* data, size_t length)
    {
        for (size_t i = 0; i < length; ++i)
        {
            uint8_t byte = static_cast<uint8_t>(data[i]);
            
            switch (currentState)
            {
                case MSP_IDLE:
                    if (byte == '$')
                    {
                        currentState = MSP_HEADER_START;
                    }
                    break;
                case MSP_HEADER_START:
                    if (byte == 'M')
                    {
                        currentState = MSP_HEADER_M;
                    }
                    else
                    {
                        currentState = MSP_IDLE;
                    }
                    break;
                case MSP_HEADER_M:
                    if (byte == '<')
                    {
                        currentState = MSP_HEADER_ARROW;
                    }
                    else
                    {
                        currentState = MSP_IDLE;
                    }
                    break;
                case MSP_HEADER_ARROW:
                    sizeToRead = static_cast<uint8_t>(byte);
                    payloadBuffer.clear();
                    payloadBuffer.reserve(sizeToRead + 2); // Розмір, команда, дані...
                    payloadBuffer.push_back(byte); // Додаємо розмір
                    currentState = MSP_PAYLOAD;
                    break;
                case MSP_PAYLOAD:
                    payloadBuffer.push_back(byte);
                    // Перевіряємо, чи отримали весь пакет
                    if (payloadBuffer.size() == sizeToRead + 2) // Розмір + команда + контрольна сума
                    {
                        uint8_t receivedChecksum = payloadBuffer.back();
                        payloadBuffer.pop_back();

                        uint8_t calculatedChecksum = 0;
                        for (uint8_t p_byte : payloadBuffer)
                        {
                            calculatedChecksum ^= p_byte;
                        }

                        uint8_t commandCode = payloadBuffer[1];
                        
                        if (receivedChecksum == calculatedChecksum)
                        {
                            std::cout << "Отримано MSP-пакет. Тип: 0x" << std::hex << (int)commandCode << ", Розмір: " << std::dec << sizeToRead << std::endl;
                            
                            std::vector<uint8_t> mspPayload;
                            // Створюємо корисне навантаження без розміру та контрольної суми
                            for(size_t j = 1; j < payloadBuffer.size(); ++j)
                            {
                                mspPayload.push_back(payloadBuffer[j]);
                            }
                            
                            // Викликаємо функцію конвертації
                            convertMspToMavlink(mspPayload, commandCode);
                        }
                        else
                        {
                            std::cerr << "Помилка контрольної суми! Отримано: 0x" << std::hex << (int)receivedChecksum << ", Очікувалося: 0x" << (int)calculatedChecksum << std::endl;
                        }
                        
                        // Повертаємось у початковий стан
                        currentState = MSP_IDLE;
                    }
                    break;
            }
        }
    }
};

#endif // MSP_CONVERTER_H