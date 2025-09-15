#include <iostream>
#include <iomanip>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>
#include <sstream>
#include "msp_converter.h"
#include "msp_protocol.h"
#include <chrono>


void sendMspV2Request(int fd, uint16_t function) {
    uint8_t frame[9];
    frame[0] = '$';
    frame[1] = 'X';   // MSPv2 marker
    frame[2] = '<';   // direction: to FC
    frame[3] = 0x00;  // flags
    frame[4] = function & 0xFF;        // function LSB
    frame[5] = (function >> 8) & 0xFF; // function MSB
    frame[6] = 0x00; // size LSB
    frame[7] = 0x00; // size MSB
    frame[8] = crc8_dvb_s2(frame + 3, 5); // CRC8 over flags+function+size

    write(fd, frame, sizeof(frame));
    tcdrain(fd);
    // std::cout << "👉 Надіслано MSPv2-запит function=0x" 
    //           << std::hex << function << std::dec << std::endl;
}

// Вивід і логування сирих байтів у HEX
void debugPrintAndLogRawHex(const char* data, ssize_t length) {
    static std::ofstream logfile("uart_raw.log", std::ios::app);

    std::ostringstream oss;
    oss << "UART RAW (" << length << " байт): ";
    for (ssize_t i = 0; i < length; i++) {
        oss << std::hex << std::setw(2) << std::setfill('0')
            << (static_cast<int>(static_cast<unsigned char>(data[i]))) << " ";
    }
    oss << std::dec << "\n";

    std::string line = oss.str();

    // Вивід у консоль
    std::cout << line;

    // Запис у лог
    if (logfile.is_open()) {
        logfile << line;
        logfile.flush();
    }
}

int setupSerial(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "Помилка: не вдалося відкрити послідовний порт " << port << std::endl;
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Помилка: не вдалося отримати атрибути послідовного порту." << std::endl;
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Помилка: не вдалося встановити атрибути послідовного порту." << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

#include <chrono>

int main() {
    std::cout << "Запуск програми-конвертера телеметрії MSP-MAVLink..." << std::endl;

    const char* serialPort = "/dev/serial0";
    int fd = setupSerial(serialPort, 115200);
    if (fd == -1) return 1;

    std::cout << "Послідовний порт " << serialPort << " відкрито." << std::endl;
    std::cout << "Очікування вхідних даних з потоку..." << std::endl;

    char buffer[512];
    MspParser parser;

    auto lastRequest = std::chrono::steady_clock::now();

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms

        int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);

        if (ready > 0 && FD_ISSET(fd, &readfds)) {
            ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                parser.parseData(buffer, bytes_read);
            }
        }

        // ⏱️ Надсилаємо запити кожні 100 мс
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRequest).count() > 100) {
            // sendMspV2Request(fd, MSP_RC); // MSP_RC
            // sendMspV2Request(fd, MSP_ATTITUDE); // MSP_ATTITUDE
            sendMspV2Request(fd, MSP_BATTERY_STATUS); // MSP_BATTERY_STATUS
            lastRequest = now;
        }
    }

    close(fd);
    return 0;
}