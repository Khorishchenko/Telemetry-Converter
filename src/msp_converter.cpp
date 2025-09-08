#include "msp_converter.h" // Обов'язково, щоб зв'язати оголошення з реалізацією

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <string>

#include <mavlink.h> // Обов'язково для роботи з MAVLink-пакетами


// Додаємо бібліотеки для роботи з мережею
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>



// Функція для обчислення контрольної суми MSP
uint8_t calculateMspChecksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum ^= byte;
    }
    return checksum;
}

// Функція для виведення MAVLink-пакета у HEX-форматі
void printHexBuffer(const uint8_t* buffer, uint16_t length) {
    std::cout << "  MAVLink-пакет (HEX): ";
    for (uint16_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

// Функція для запису MAVLink-пакета у файл
void writeToFile(const std::string& filename, const uint8_t* buffer, uint16_t length) {
    std::ofstream outfile(filename, std::ios::binary | std::ios::app);
    if (outfile.is_open()) {
        outfile.write(reinterpret_cast<const char*>(buffer), length);
        outfile.close();
        std::cout << "  Записано " << length << " байт у файл " << filename << std::endl;
    } else {
        std::cerr << "Помилка: не вдалося відкрити файл для запису: " << filename << std::endl;
    }
}

// Функція для перетворення MSP-даних в MAVLink-пакети
void convertMspToMavlink(const std::vector<uint8_t>& mspPayload, uint8_t commandCode) {
    mavlink_message_t mavlink_msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    switch (commandCode) {
        case 106: { // MSP_RAW_GPS
            if (mspPayload.size() >= 18) {
                // Широта (int32_t) * 10 000 000
                int32_t lat = static_cast<int32_t>(mspPayload[0] | (mspPayload[1] << 8) | (mspPayload[2] << 16) | (mspPayload[3] << 24));
                // Довгота (int32_t) * 10 000 000
                int32_t lon = static_cast<int32_t>(mspPayload[4] | (mspPayload[5] << 8) | (mspPayload[6] << 16) | (mspPayload[7] << 24));
                // Висота (int32_t) (cm)
                int32_t alt = static_cast<int32_t>(mspPayload[8] | (mspPayload[9] << 8) | (mspPayload[10] << 16) | (mspPayload[11] << 24));
                // Швидкість (uint16_t) (cm/s)
                uint16_t speed = static_cast<uint16_t>(mspPayload[12] | (mspPayload[13] << 8));
                // Кількість супутників (uint8_t)
                uint8_t numSat = mspPayload[14];
                // Фікс (uint8_t)
                uint8_t fixType = mspPayload[15];

                std::cout << "  Парсинг MSP_RAW_GPS: Широта: " << lat / 10000000.0 << "°, Довгота: " << lon / 10000000.0 << "°, Висота: " << alt / 100.0f << " м" << std::endl;
                std::cout << "  Швидкість: " << speed / 100.0f << " м/с, Супутників: " << static_cast<int>(numSat) << ", Фікс: " << static_cast<int>(fixType) << std::endl;

                // Створюємо MAVLink-пакет GPS_RAW_INT
                mavlink_msg_gps_raw_int_pack(1, 1, &mavlink_msg,
                                             0, // час (мкс)
                                             fixType,
                                             lat,
                                             lon,
                                             alt,
                                             0, // eph
                                             0, // epv
                                             speed,
                                             0, // cog
                                             numSat,
                                             0, 0, 0, 0, 0, 0);

                                             
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink GPS_RAW_INT-пакет. Розмір: " << len << " байт." << std::endl;
                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);
                
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        }
        case 108: { // MSP_ATTITUDE
            if (mspPayload.size() >= 6) {
                // Витягуємо дані
                // Крен: байти 0, 1
                int16_t roll = static_cast<int16_t>(mspPayload[0] | (mspPayload[1] << 8));
                // Тангаж: байти 2, 3
                int16_t pitch = static_cast<int16_t>(mspPayload[2] | (mspPayload[3] << 8));
                // Курс: байти 4, 5
                int16_t yaw = static_cast<int16_t>(mspPayload[4] | (mspPayload[5] << 8));
                
                // Перетворюємо в радіани для MAVLink
                float roll_rad = roll * (M_PI / 1800.0f);
                float pitch_rad = pitch * (M_PI / 1800.0f);
                float yaw_rad = yaw * (M_PI / 180.0f);

                std::cout << "  Парсинг MSP_ATTITUDE: Крен: " << roll / 10.0f << "°, Тангаж: " << pitch / 10.0f << "°, Курс: " << yaw << "°" << std::endl;

                // Створюємо MAVLink-пакет ATTITUDE
                mavlink_msg_attitude_pack(1, 1, &mavlink_msg,
                                        0, // час в мілісекундах
                                        roll_rad, pitch_rad, yaw_rad,
                                        0, 0, 0);

                // Заповнюємо буфер
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink ATTITUDE-пакет. Розмір: " << len << " байт." << std::endl;
                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);

                
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);

            }
            break;
        }
        case 105: { // MSP_RC
            if (mspPayload.size() >= 16) {
                // RC-канали передаються як 16-бітові цілі числа.
                // Ми витягуємо перші 8 каналів
                std::vector<uint16_t> channels(8);
                for (size_t i = 0; i < 8; ++i) {
                    channels[i] = mspPayload[i * 2] | (mspPayload[i * 2 + 1] << 8);
                }

                std::cout << "  Парсинг MSP_RC (Канали): ";
                for (size_t i = 0; i < 8; ++i) {
                    std::cout << "CH" << i + 1 << ": " << channels[i] << "  ";
                }
                std::cout << std::endl;

                // Створюємо MAVLink-пакет RC_CHANNELS
                mavlink_msg_rc_channels_pack(1, 1, &mavlink_msg,
                                             0, // час в мілісекундах
                                             8, // Кількість каналів
                                             channels[0], channels[1], channels[2], channels[3],
                                             channels[4], channels[5], channels[6], channels[7],
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); // Заповнюємо нулями решту каналів
                
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink RC_CHANNELS-пакет. Розмір: " << len << " байт." << std::endl;
                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);


                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);

            }
            break;
        }
        case 107: { // MSP_BATTERY_STATUS
            if (mspPayload.size() >= 7) {
                // Напруга (мВ)
                uint16_t voltage = mspPayload[0] | (mspPayload[1] << 8);
                // Струм (мА)
                int16_t current = mspPayload[2] | (mspPayload[3] << 8);
                // Ємність (мАг)
                uint16_t mahDrawn = mspPayload[4] | (mspPayload[5] << 8);

                std::cout << "  Парсинг MSP_BATTERY_STATUS: Напруга: " << voltage / 100.0f << " В, Струм: " << current / 10.0f << " A, Спожито: " << mahDrawn << " мАг" << std::endl;

                // Створюємо масив для напруг, як того вимагає MAVLink
                uint16_t voltages[10] = {0};
                voltages[0] = voltage; // Записуємо нашу напругу в перший елемент масиву

                // Створюємо MAVLink-пакет BATTERY_STATUS
                mavlink_msg_battery_status_pack(1, 1, &mavlink_msg,
                                                1, // id
                                                MAV_BATTERY_FUNCTION_ALL,
                                                MAV_BATTERY_TYPE_LIPO,
                                                0, // temperature (використовуємо 0, якщо не маємо даних)
                                                voltages,
                                                current,
                                                -1, // battery_remaining
                                                0, 0, 0,
                                                0, 0, 0, 0); // current_consumed

                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink BATTERY_STATUS-пакет. Розмір: " << len << " байт." << std::endl;
                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);

                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        }
        default:
            std::cout << "  Невідомий тип MSP-повідомлення: 0x" << std::hex << (int)commandCode << std::endl;
            break;
    }
}

// Глобальна функція для парсингу MSP
void parseMspData(FILE* input) {
    enum {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_M,
        MSP_HEADER_ARROW,
        MSP_HEADER_V1,
        MSP_PAYLOAD
    } state = MSP_IDLE;

    std::vector<uint8_t> payload;
    uint16_t sizeToRead = 0;
    
    int byte_int;
    while ((byte_int = fgetc(input)) != EOF) {
        uint8_t byte = static_cast<uint8_t>(byte_int);
        switch (state) {
            case MSP_IDLE:
                if (byte == '$') {
                    state = MSP_HEADER_START;
                }
                break;
            case MSP_HEADER_START:
                if (byte == 'M') {
                    state = MSP_HEADER_M;
                } else {
                    state = MSP_IDLE;
                }
                break;
            case MSP_HEADER_M:
                if (byte == '<') {
                    state = MSP_HEADER_ARROW;
                } else {
                    state = MSP_IDLE;
                }
                break;
            case MSP_HEADER_ARROW:
                sizeToRead = static_cast<uint8_t>(byte);
                payload.reserve(sizeToRead + 1);
                payload.clear();
                payload.push_back(byte); // Додаємо розмір в корисне навантаження
                state = MSP_HEADER_V1;
                break;
            case MSP_HEADER_V1:
                payload.push_back(byte);
                if (payload.size() == sizeToRead + 2) { // Розмір + команда + контрольна сума
                    uint8_t receivedChecksum = payload.back();
                    payload.pop_back();

                    uint8_t calculatedChecksum = 0;
                    for (uint8_t p_byte : payload) {
                        calculatedChecksum ^= p_byte;
                    }
                    
                    uint8_t commandCode = payload[1];
                    
                    if (receivedChecksum == calculatedChecksum) {
                        std::cout << "Отримано MSP-пакет. Тип: 0x" << std::hex << (int)commandCode << ", Розмір: " << std::dec << sizeToRead << std::endl;
                        // Створюємо корисне навантаження без розміру та контрольної суми
                        std::vector<uint8_t> mspPayload;
                        for(size_t i = 1; i < payload.size(); ++i) {
                            mspPayload.push_back(payload[i]);
                        }
                        convertMspToMavlink(mspPayload, commandCode);
                    } else {
                        std::cerr << "Помилка контрольної суми! Отримано: 0x" << std::hex << (int)receivedChecksum << ", Очікувалося: 0x" << (int)calculatedChecksum << std::endl;
                    }

                    state = MSP_IDLE;
                }
                break;
        }
    }
}


// Функція для відправки MAVLink-пакету по UDP
void sendMavlinkPacketOverUdp(const uint8_t* buffer, uint16_t length, const std::string& ip_address, int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Помилка: не вдалося створити UDP-сокет." << std::endl;
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr);

    if (sendto(sock, buffer, length, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Помилка: не вдалося відправити UDP-пакет." << std::endl;
    } else {
        std::cout << "  Відправлено " << length << " байт по UDP на " << ip_address << ":" << port << std::endl;
    }

    close(sock);
}