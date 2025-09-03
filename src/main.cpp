//================================================================================================
// Базова версія для парсингу MSP-даних з файлу
//================================================================================================

// #include <iostream>
// #include <fstream>
// #include <vector>

// // Підключаємо MAVLink
// #include <mavlink.h>

// Максимальний розмір пакета MAVLink
// #define MAVLINK_MAX_PACKET_LEN 280

// // Функція для парсингу MSP-даних з файлу
// void parseMspDataFromFile(const std::string& filename) {
//     // Відкриваємо файл у бінарному режимі
//     std::ifstream file(filename, std::ios::binary);

//     if (!file.is_open()) {
//         std::cerr << "Помилка: не вдалося відкрити файл " << filename << std::endl;
//         return;
//     }

//     std::cout << "Читання даних з файлу: " << filename << std::endl;

//     // TODO: Тут буде реалізована логіка парсингу та перетворення
//     // Зараз просто читаємо і виводимо байти
//     char byte;
//     while (file.get(byte)) {
//         std::cout << std::hex << (int)(unsigned char)byte << " ";
//     }
//     std::cout << std::endl;

//     file.close();
// }

// int main(int argc, char** argv) {
//     // Перевіряємо, чи був переданий шлях до файлу
//     if (argc < 2) {
//         std::cerr << "Використання: " << argv[0] << " <шлях_до_файлу_з_даними_MSP>" << std::endl;
//         return 1;
//     }

//     // Викликаємо функцію для парсингу
//     parseMspDataFromFile(argv[1]);

//     return 0;
// }



//================================================================================================
// Базова версія для парсингу MSP-даних з файлу
//================================================================================================


// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <cstdint>
// #include <iomanip>

// // Підключаємо MAVLink
// #include <mavlink.h>

// // Максимальний розмір пакета MAVLink
// #define MAVLINK_MAX_PACKET_LEN 280

// // Функція для обчислення контрольної суми MSP
// uint8_t calculateMspChecksum(const std::vector<uint8_t>& data) {
//     uint8_t checksum = 0;
//     for (uint8_t byte : data) {
//         checksum ^= byte;
//     }
//     return checksum;
// }

// // Функція для парсингу MSP-даних з файлу
// void parseMspDataFromFile(const std::string& filename) {
//     std::ifstream file(filename, std::ios::binary);
//     if (!file.is_open()) {
//         std::cerr << "Помилка: не вдалося відкрити файл " << filename << std::endl;
//         return;
//     }

//     std::cout << "Читання та парсинг MSP-даних з файлу: " << filename << std::endl;

//     enum State {
//         STATE_IDLE,
//         STATE_HEADER_M,
//         STATE_HEADER_DIR,
//         STATE_HEADER_SIZE,
//         STATE_HEADER_CODE,
//         STATE_PAYLOAD,
//         STATE_CHECKSUM
//     };

//     State currentState = STATE_IDLE;
//     std::vector<uint8_t> rxBuffer;
//     uint8_t dataSize = 0;

//     char byteChar;
//     while (file.get(byteChar)) {
//         uint8_t byte = static_cast<uint8_t>(byteChar);

//         switch (currentState) {
//             case STATE_IDLE:
//                 if (byte == '$') {
//                     currentState = STATE_HEADER_M;
//                 }
//                 break;
//             case STATE_HEADER_M:
//                 if (byte == 'M') {
//                     currentState = STATE_HEADER_DIR;
//                 } else {
//                     currentState = STATE_IDLE;
//                 }
//                 break;
//             case STATE_HEADER_DIR:
//                 if (byte == '<') {
//                     currentState = STATE_HEADER_SIZE;
//                 } else {
//                     currentState = STATE_IDLE;
//                 }
//                 break;
//             case STATE_HEADER_SIZE:
//                 dataSize = byte;
//                 rxBuffer.clear(); // Очищаємо буфер для нового пакету
//                 rxBuffer.push_back(byte);
//                 currentState = STATE_HEADER_CODE;
//                 break;
//             case STATE_HEADER_CODE:
//                 rxBuffer.push_back(byte);
//                 if (dataSize > 0) {
//                     currentState = STATE_PAYLOAD;
//                 } else {
//                     currentState = STATE_CHECKSUM;
//                 }
//                 break;
//             case STATE_PAYLOAD:
//                 rxBuffer.push_back(byte);
//                 if (rxBuffer.size() == 2 + dataSize) {
//                     currentState = STATE_CHECKSUM;
//                 }
//                 break;
//             case STATE_CHECKSUM: {
//                 uint8_t receivedChecksum = byte;
//                 uint8_t calculatedChecksum = calculateMspChecksum(rxBuffer);

//                 if (receivedChecksum == calculatedChecksum) {
//                     std::cout << "Отримано MSP-пакет. Тип: 0x" << std::hex << std::setw(2) << std::setfill('0') << (int)rxBuffer[1] << ", Розмір: " << std::dec << (int)rxBuffer[0] << std::endl;
//                     // TODO: Тут буде логіка перетворення в MAVLink
//                 } else {
//                     std::cerr << "Помилка контрольної суми! Отримано: 0x" << std::hex << (int)receivedChecksum << ", Очікувалося: 0x" << (int)calculatedChecksum << std::endl;
//                 }
                
//                 currentState = STATE_IDLE;
//                 break;
//             }
//         }
//     }
//     file.close();
// }

// int main(int argc, char** argv) {
//     if (argc < 2) {
//         std::cerr << "Використання: " << argv[0] << " <шлях_до_файлу_з_даними_MSP>" << std::endl;
//         return 1;
//     }
//     parseMspDataFromFile(argv[1]);
//     return 0;
// }






//================================================================================================
// Розширена версія з перетворенням MSP в MAVLink
//================================================================================================





#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <cmath> // Для використання M_PI

// Підключаємо MAVLink
#include <mavlink.h>


#include "msp_converter.h"

// Максимальний розмір пакета MAVLink
#define MAVLINK_MAX_PACKET_LEN 280


// Функція для парсингу MSP-даних з файлу
void parseMspDataFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Помилка: не вдалося відкрити файл " << filename << std::endl;
        return;
    }

    std::cout << "Читання та парсинг MSP-даних з файлу: " << filename << std::endl;

    enum State {
        STATE_IDLE,
        STATE_HEADER_M,
        STATE_HEADER_DIR,
        STATE_HEADER_SIZE,
        STATE_HEADER_CODE,
        STATE_PAYLOAD,
        STATE_CHECKSUM
    };

    State currentState = STATE_IDLE;
    std::vector<uint8_t> rxBuffer;
    uint8_t dataSize = 0;

    char byteChar;
    while (file.get(byteChar)) {
        uint8_t byte = static_cast<uint8_t>(byteChar);

        switch (currentState) {
            case STATE_IDLE:
                if (byte == '$') {
                    currentState = STATE_HEADER_M;
                }
                break;
            case STATE_HEADER_M:
                if (byte == 'M') {
                    currentState = STATE_HEADER_DIR;
                } else {
                    currentState = STATE_IDLE;
                }
                break;
            case STATE_HEADER_DIR:
                if (byte == '<') {
                    currentState = STATE_HEADER_SIZE;
                } else {
                    currentState = STATE_IDLE;
                }
                break;
            case STATE_HEADER_SIZE:
                dataSize = byte;
                rxBuffer.clear();
                rxBuffer.push_back(byte);
                currentState = STATE_HEADER_CODE;
                break;
            case STATE_HEADER_CODE:
                rxBuffer.push_back(byte);
                if (dataSize > 0) {
                    currentState = STATE_PAYLOAD;
                } else {
                    currentState = STATE_CHECKSUM;
                }
                break;
            case STATE_PAYLOAD:
                rxBuffer.push_back(byte);
                if (rxBuffer.size() == 2 + dataSize) {
                    currentState = STATE_CHECKSUM;
                }
                break;
            case STATE_CHECKSUM: {
                uint8_t receivedChecksum = byte;
                std::vector<uint8_t> checksumData(rxBuffer.begin(), rxBuffer.end());
                uint8_t calculatedChecksum = calculateMspChecksum(checksumData);

                if (receivedChecksum == calculatedChecksum) {
                    std::cout << "Отримано MSP-пакет. Тип: 0x" << std::hex << std::setw(2) << std::setfill('0') << (int)rxBuffer[1] << ", Розмір: " << std::dec << (int)rxBuffer[0] << std::endl;
                    
                    // Викликаємо функцію для перетворення в MAVLink
                    std::vector<uint8_t> mspPayload;
                    for (size_t i = 2; i < rxBuffer.size(); ++i) {
                        mspPayload.push_back(rxBuffer[i]);
                    }
                    convertMspToMavlink(mspPayload, rxBuffer[1]);

                } else {
                    std::cerr << "Помилка контрольної суми! Отримано: 0x" << std::hex << (int)receivedChecksum << ", Очікувалося: 0x" << (int)calculatedChecksum << std::endl;
                }
                
                currentState = STATE_IDLE;
                break;
            }
        }
    }
    file.close();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Використання: " << argv[0] << " <шлях_до_файлу_з_даними_MSP>" << std::endl;
        return 1;
    }
    parseMspDataFromFile(argv[1]);
    return 0;
}