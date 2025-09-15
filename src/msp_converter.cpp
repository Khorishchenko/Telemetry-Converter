#include "msp_converter.h"
#include "msp_protocol.h"
#include <mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <cmath>
#include <iomanip>


// ==========================================================
// CRC-8/DVB-S2 (poly=0xD5, init=0x00, no reflection, xorout=0x00)
// ==========================================================
uint8_t crc8_dvb_s2(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            else crc = static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}


uint8_t calculateMspChecksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum ^= byte;
    }
    return checksum;
}

void printHexBuffer(const uint8_t* buffer, uint16_t length) {
    std::cout << "  MAVLink-пакет (HEX): ";
    for (uint16_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

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

void convertMspToMavlink(const std::vector<uint8_t>& mspPayload, uint16_t commandCode) {
    mavlink_message_t mavlink_msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    switch (commandCode) {
        case MSP_RAW_GPS: // MSP_RAW_GPS
            if (mspPayload.size() >= 16) {
                int32_t lat = (mspPayload[0] | (mspPayload[1] << 8) | (mspPayload[2] << 16) | (mspPayload[3] << 24));
                int32_t lon = (mspPayload[4] | (mspPayload[5] << 8) | (mspPayload[6] << 16) | (mspPayload[7] << 24));
                int32_t alt = (mspPayload[8] | (mspPayload[9] << 8) | (mspPayload[10] << 16) | (mspPayload[11] << 24));
                uint16_t speed = (mspPayload[12] | (mspPayload[13] << 8));
                uint8_t numSat = mspPayload[14];
                uint8_t fixType = mspPayload[15];

                std::cout << "  Парсинг MSP_RAW_GPS: Широта: " << lat / 10000000.0 << "°, Довгота: " << lon / 10000000.0 << "°, Висота: " << alt / 100.0f << " м" << std::endl;
                std::cout << "  Швидкість: " << speed / 100.0f << " м/с, Супутників: " << static_cast<int>(numSat) << ", Фікс: " << static_cast<int>(fixType) << std::endl;

                mavlink_msg_gps_raw_int_pack(1, 1, &mavlink_msg, 0, fixType, lat, lon, alt, 0, 0, speed, 0, numSat, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);

                std::cout << "  Згенеровано MAVLink GPS_RAW_INT-пакет. Розмір: " << len << " байт." << std::endl;

                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        case MSP_ATTITUDE: // MSP_ATTITUDE
            if (mspPayload.size() >= 6) {
                int16_t roll = (mspPayload[0] | (mspPayload[1] << 8));
                int16_t pitch = (mspPayload[2] | (mspPayload[3] << 8));
                int16_t yaw = (mspPayload[4] | (mspPayload[5] << 8));
                float roll_rad = roll * (M_PI / 1800.0f);
                float pitch_rad = pitch * (M_PI / 1800.0f);
                float yaw_rad = yaw * (M_PI / 1800.0f);

                std::cout << "  Парсинг " << COLOR_GREEN << " MSP_ATTITUDE: " << COLOR_RESET << "Крен: " << roll / 10.0f << "°, Тангаж: " << pitch / 10.0f << "°, Курс: " << yaw << "°" << std::endl;

                mavlink_msg_attitude_pack(1, 1, &mavlink_msg, 0, roll_rad, pitch_rad, yaw_rad, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink ATTITUDE-пакет. Розмір: " << len << " байт." << std::endl;

                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        case MSP_RC: // MSP_RC
            if (mspPayload.size() >= 16) {
                std::vector<uint16_t> channels(8);
                for (size_t i = 0; i < 8; ++i) {
                    channels[i] = (mspPayload[i * 2] | (mspPayload[i * 2 + 1] << 8));
                }

                std::cout << "  Парсинг MSP_RC (Канали): ";
                for (size_t i = 0; i < 8; ++i) {
                    std::cout << "CH" << i + 1 << ": " << channels[i] << "  ";
                }
                std::cout << std::endl;

                mavlink_msg_rc_channels_pack(1, 1, &mavlink_msg, 0, 8, channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6], channels[7], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);
                std::cout << "  Згенеровано MAVLink RC_CHANNELS-пакет. Розмір: " << len << " байт." << std::endl;

                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        case MSP_BATTERY_STATUS: // MSP_BATTERY_STATUS
            if (mspPayload.size() >= 6) {
                uint16_t voltage = (mspPayload[0] | (mspPayload[1] << 8));
                int16_t current = (mspPayload[2] | (mspPayload[3] << 8));
                uint16_t mahDrawn = (mspPayload[4] | (mspPayload[5] << 8));
                uint16_t voltages[10] = {voltage, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                std::cout << "  Парсинг " << COLOR_GREEN << " MSP_BATTERY_STATUS:" << COLOR_RESET << " Напруга: " << voltage / 100.0f << " В, Струм: " << current / 10.0f << " A, Спожито: " << mahDrawn << " мАг" << std::endl;

                mavlink_msg_battery_status_pack(1, 1, &mavlink_msg, 1, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 0, voltages, current, -1, 0, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &mavlink_msg);

                std::cout << "  Згенеровано MAVLink BATTERY_STATUS-пакет. Розмір: " << len << " байт." << std::endl;

                printHexBuffer(buf, len);
                writeToFile("msp_telemetry.bin", buf, len);
                sendMavlinkPacketOverUdp(buf, len, "127.0.0.1", 14550);
            }
            break;
        case 0x1208:
            std::cout << "Отримano MSPv2 пакет 0x1208. Розмір: " << mspPayload.size() << " байт." << std::endl;
            std::cout << "Вміст mspPayload (HEX): ";
            for (size_t i = 0; i < mspPayload.size(); ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mspPayload[i]) << " ";
            }
            std::cout << std::dec << std::endl;
            break;
        default:
            std::cout << "  Невідомий тип MSP-повідомлення: 0x" << std::hex << (int)commandCode << std::endl;
            std::cout << "  Payload (" << std::dec << mspPayload.size() << " байт): ";
            for (size_t i = 0; i < mspPayload.size(); ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mspPayload[i]) << " ";
            }
            std::cout << std::dec << std::endl;
            break;
    }
}

// void MspParser::parseData(const char* data, size_t length) {
//     for (size_t i = 0; i < length; ++i) {
//         uint16_t byte = static_cast<uint16_t>(data[i]);

//         switch (currentState) {
//             case MSP_IDLE: {
//                 if (byte == '$') {
//                     std::cout << "⚡️Початок нового MSP-пакету знайдено: '$'" << std::endl;
//                     currentState = MSP_HEADER_START;
//                     payloadBuffer.clear();
//                     flags = 0;
//                     sizeLSB = 0;
//                     sizeMSB = 0;
//                     cmdLSB = 0;
//                     cmdMSB = 0;
//                 }
//                 break;
//             }
//             case MSP_HEADER_START: {
//                 if (byte == 'X') {
//                     currentState = MSP_HEADER_M;
//                     std::cout << "⚡️Знайдено 'X' після '$'" << std::endl;
//                 } else {
//                     currentState = MSP_IDLE;
//                 }
//                 break;
//             }
//             case MSP_HEADER_M: {
//                 if (byte == '>' || byte == '<') {
//                     currentState = MSP_HEADER_ARROW;
//                     payloadBuffer.clear();
//                     payloadBuffer.push_back(byte); // flags
//                     std::cout << "⚡️Знайдено початок MSPv2-пакету: $X" << byte << std::endl;
//                 } else {
//                     currentState = MSP_IDLE;
//                 }
//                 break;
//             }
//             case MSP_HEADER_ARROW: {
//                 flags = byte;
//                 payloadBuffer.push_back(byte); // sizeLSB
//                 currentState = MSP_HEADER_SIZE_LSB;
//                 std::cout << "⚡️Флаги MSPv2: 0x" << std::hex << (int)flags << std::dec << std::endl;
//                 break;
//             }
//             case MSP_HEADER_SIZE_LSB: {
//                 sizeLSB = byte;
//                 payloadBuffer.push_back(byte); // sizeMSB
//                 currentState = MSP_HEADER_SIZE_MSB;
//                 std::cout << "⚡️Розмір payload LSB: " << (int)sizeLSB << std::endl;
//                 break;
//             }
//             case MSP_HEADER_SIZE_MSB: {
//                 sizeMSB = byte;
//                 payloadBuffer.push_back(byte); // cmdLSB
//                 currentState = MSP_HEADER_CMD_LSB;
//                 std::cout << "⚡️Розмір payload MSB: " << (int)sizeMSB << std::endl;
//                 break;
//             }
//             case MSP_HEADER_CMD_LSB: {
//                 cmdLSB = byte;
//                 payloadBuffer.push_back(byte); // cmdMSB
//                 currentState = MSP_HEADER_CMD_MSB;
//                 std::cout << "⚡️Команда MSP LSB: 0x" << std::hex << (int)cmdLSB << std::dec << std::endl;
//                 break;
//             }
//             case MSP_HEADER_CMD_MSB: {
//                 cmdMSB = byte;
//                 payloadBuffer.push_back(byte); // payload starts next
//                 payloadBytesRead = 0;
//                 uint16_t payloadSize = (static_cast<uint16_t>(sizeMSB) << 8) | sizeLSB;
//                 if (payloadSize == 0) {
//                     currentState = MSP_CHECKSUM;
//                     std::cout << "⚡️Payload розмір 0, переходимо до контрольної суми." << std::endl;
//                 } else {
//                     currentState = MSP_PAYLOAD;
//                     std::cout << "⚡️Команда MSP MSB: 0x" << std::hex << (int)cmdMSB << std::dec << std::endl;
//                     std::cout << "⚡️Очікуваний розмір payload: " << payloadSize << " байт." << std::endl;
//                 }
//                 break;
//             }
//             case MSP_PAYLOAD: {
//                 payloadBuffer.push_back(byte);
//                 ++payloadBytesRead;
//                 uint16_t payloadSize = (static_cast<uint16_t>(sizeMSB) << 8) | sizeLSB;
//                 if (payloadBytesRead >= payloadSize) {
//                     currentState = MSP_CHECKSUM;
//                 }
//                 break;
//             }
//             case MSP_CHECKSUM: {
//                 // CRC8-DVB-S2 по всіх байтах після '>'/<' включно до кінця payload (без CRC)
//                 uint8_t recvCRC = byte;
//                 uint8_t calcCRC = crc8_dvb_s2(payloadBuffer.data(), payloadBuffer.size());
//                 uint16_t function = (static_cast<uint16_t>(cmdMSB) << 8) | cmdLSB;
//                 uint16_t payloadSize = (static_cast<uint16_t>(sizeMSB) << 8) | sizeLSB;

//                 if (recvCRC == calcCRC) {
//                     std::cout << "✅ Отримано MSPv2-пакет. Function: 0x" << std::hex << function
//                               << ", Розмір: " << std::dec << payloadSize << std::endl;
//                     // payloadBuffer: [flags, sizeLSB, sizeMSB, cmdLSB, cmdMSB, payload...]
//                     std::vector<uint8_t> mspPayload;
//                     if (payloadSize > 0 && payloadBuffer.size() >= 5 + payloadSize) {
//                         mspPayload.assign(payloadBuffer.begin() + 5, payloadBuffer.begin() + 5 + payloadSize);
//                     }
//                     convertMspToMavlink(mspPayload, function);
//                     currentState = MSP_IDLE;
//                 } else {
//                     std::cerr << "❌ Помилка контрольної суми MSPv2! Отримано: 0x" << std::hex << (int)recvCRC
//                               << ", Очікувалося: 0x" << (int)calcCRC << std::dec << std::endl;
//                     currentState = MSP_IDLE;
//                 }
//                 break;
//             }
//             default:
//                 currentState = MSP_IDLE;
//                 break;
//         }
//     }
// }




void MspParser::parseData(const char* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = static_cast<uint8_t>(data[i]);

        switch (currentState) {
            case MSP_IDLE:
                // Очікуємо початок пакета
                if (byte == '$') {
                    currentState = MSP_HEADER_START;
                    headerBuffer.clear();
                    payloadBuffer.clear();
                    headerBuffer.push_back(byte);
                }
                break;

            case MSP_HEADER_START:
                // Збираємо другий байт заголовка ('M' або 'X')
                headerBuffer.push_back(byte);
                if (byte == 'M' || byte == 'X') {
                    currentState = MSP_HEADER_M;
                } else {
                    currentState = MSP_IDLE;
                }
                break;

            case MSP_HEADER_M:
                // Збираємо третій байт заголовка ('<' або '>')
                headerBuffer.push_back(byte);
                if (headerBuffer[1] == 'M') {
                    // MSPv1
                    if (byte == '<' || byte == '>') {
                        currentState = MSP_PAYLOAD;
                        payloadBuffer.clear();
                    } else {
                        currentState = MSP_IDLE;
                    }
                } else if (headerBuffer[1] == 'X') {
                    // MSPv2
                    if (byte == '<' || byte == '>') {
                        currentState = MSP_PAYLOAD;
                        payloadBuffer.clear();
                    } else {
                        currentState = MSP_IDLE;
                    }
                } else {
                    currentState = MSP_IDLE;
                }
                break;

            case MSP_PAYLOAD:
                // Збираємо payload
                payloadBuffer.push_back(byte);

                // MSPv1 парсинг
                if (headerBuffer[1] == 'M') {
                    if (payloadBuffer.size() >= 3) {
                        uint8_t size = payloadBuffer[0];
                        if (payloadBuffer.size() == (size + 3)) {
                            uint8_t cmd = payloadBuffer[1];
                            uint8_t calc = size ^ cmd;
                            for (size_t j = 0; j < size; j++)
                                calc ^= payloadBuffer[2 + j];
                            uint8_t recv = payloadBuffer.back();

                            std::cout << "✅ Отримано MSPv1-пакет. Команда: 0x" << std::hex << (int)cmd
                                      << ", Розмір: " << std::dec << (int)size << std::endl;
                            std::cout << "  Вміст (HEX): ";
                            for (auto b : payloadBuffer)
                                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                            std::cout << std::dec << std::endl;

                            if (recv == calc) {
                                std::vector<uint8_t> mspPayload(payloadBuffer.begin() + 2,
                                                                payloadBuffer.end() - 1);
                                convertMspToMavlink(mspPayload, cmd);
                            } else {
                                std::cerr << "❌ Помилка контрольної суми MSPv1! Отримано: 0x"
                                          << std::hex << (int)recv
                                          << ", Очікувалось: 0x" << (int)calc
                                          << std::dec << std::endl;
                            }
                            payloadBuffer.clear();
                            currentState = MSP_IDLE;
                        }
                    }
                }
                // MSPv2 парсинг
                else if (headerBuffer[1] == 'X') {
                    if (payloadBuffer.size() >= 5) {
                        uint8_t flags = payloadBuffer[0];
                        uint16_t function = payloadBuffer[1] | (payloadBuffer[2] << 8);
                        uint16_t size = payloadBuffer[3] | (payloadBuffer[4] << 8);

                        size_t totalNeeded = 5 + size + 1; // flags+func(2)+size(2)+payload+crc
                        if (payloadBuffer.size() == totalNeeded) {
                            uint8_t calc = crc8_dvb_s2(payloadBuffer.data(), 5 + size);
                            uint8_t recv = payloadBuffer.back();

                            std::cout << "✅ Отримано MSPv2-пакет. Функція: 0x" << std::hex << function
                                      << ", Розмір: " << std::dec << size << std::endl;
                            std::cout << "  Вміст (HEX): ";
                            for (auto b : payloadBuffer)
                                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                            std::cout << std::dec << std::endl;

                            if (recv == calc) {
                                std::vector<uint8_t> mspPayload(payloadBuffer.begin() + 5,
                                                                payloadBuffer.end() - 1);
                                convertMspToMavlink(mspPayload, function);
                            } else {
                                std::cerr << "❌ Помилка контрольної суми MSPv2! Отримано: 0x"
                                          << std::hex << (int)recv
                                          << ", Очікувалось: 0x" << (int)calc
                                          << std::dec << std::endl;
                            }
                            payloadBuffer.clear();
                            currentState = MSP_IDLE;
                        }
                    }
                }
                break;

            default:
                currentState = MSP_IDLE;
                break;
        }
    }
}