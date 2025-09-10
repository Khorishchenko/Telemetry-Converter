#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// Підключаємо MAVLink
#include <mavlink.h>
// Підключаємо наш заголовочний файл з функціями конвертера
#include "msp_converter.h"

// Максимальний розмір пакета MAVLink (визначений в msp_converter.cpp)
#define MAVLINK_MAX_PACKET_LEN 280


// функція для налаштування послідовного порту
int setupSerial(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Помилка: не вдалося відкрити послідовний порт " << port << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Помилка: не вдалося отримати атрибути послідовного порту." << std::endl;
        return -1;
    }

    // Встановлюємо швидкість передачі
    cfsetospeed(&tty, B115200); 
    // cfsetispeed(&tty, B115200);

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
        return -1;
    }

    return fd;
}

// Головна функція для роботи з послідовним портом
int main(int argc, char* argv[]) {
    // std::cout << "Запуск програми-конвертера телеметрії MSP-MAVLink..." << std::endl;

    // const char* serialPort = "/dev/serial0";
    // int baudrate = 115200;

    // int fd = setupSerial(serialPort, baudrate);
    // if (fd == -1) {
    //     return 1;
    // }

    // std::cout << "Послідовний порт " << serialPort << " відкрито." << std::endl;

    // // Цикл для очікування вхідних даних
    // std::cout << "Очікування вхідних даних з потоку..." << std::endl;


    // char buffer[256];
    // MspParser parser;
    // Головний нескінченний цикл
    // while (true) 
    // {
        // fd_set readfds;
        // FD_ZERO(&readfds);
        // FD_SET(fd, &readfds);
        

        // // Налаштування тайм-ауту (наприклад, 1 секунда)
        // struct timeval timeout;
        // timeout.tv_sec = 1;
        // timeout.tv_usec = 0;

        // // Виклик select для перевірки наявності даних
        // int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);

        // if (ready > 0 && FD_ISSET(fd, &readfds)) {
        //     // Дані доступні для читання
        //     ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            
        //     if (bytes_read > 0) {
        //         // Виводимо отримані дані
        //         // std::cout.write(buffer, bytes_read);

        //         // І передаємо їх для парсингу
        //         // parser.parseData(buffer, bytes_read);
        //     }
        // } 
        // else {
        //     std::cout << "Все ще очікуємо..." << std::endl;
        // }
    // }
    // close(fd);

    return 0;
}