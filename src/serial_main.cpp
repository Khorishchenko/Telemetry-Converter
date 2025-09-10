#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>
#include "msp_converter.h"

int setupSerial(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Помилка: не вдалося відкрити послідовний порт " << port << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Помилка: не вдалося отримати атрибути послідовного порту." << std::endl;
        close(fd);
        return -1;
    }

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

int main() {
    std::cout << "Запуск програми-конвертера телеметрії MSP-MAVLink..." << std::endl;

    const char* serialPort = "/dev/serial0";
    int fd = setupSerial(serialPort, 115200);
    if (fd == -1) return 1;

    std::cout << "Послідовний порт " << serialPort << " відкрито." << std::endl;
    std::cout << "Очікування вхідних даних з потоку..." << std::endl;

    char buffer[512];
    MspParser parser;
    std::vector<char> dataBuffer;

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (ready > 0 && FD_ISSET(fd, &readfds)) {
            ssize_t bytes_read = read(fd, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                dataBuffer.insert(dataBuffer.end(), buffer, buffer + bytes_read);
                size_t offset = 0;
                while (offset < dataBuffer.size()) {
                    parser.parseData(dataBuffer.data() + offset, dataBuffer.size() - offset);
                    std::cout.write(dataBuffer.data() + offset, dataBuffer.size() - offset);
                    if (parser.getState() == parser.getMspChecksum() && parser.isPacketComplete()) {
                        dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + offset + parser.getPayloadSize() + 3);
                        offset = 0;
                    } else {
                        offset++;
                    }
                }
            }
        } else if (ready == 0) {
            std::cout << "Все ще очікуємо..." << std::endl;
        } else {
            std::cerr << "Помилка select" << std::endl;
            break;
        }
    }

    close(fd);
    return 0;
}