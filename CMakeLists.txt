# Визначаємо мінімальну версію CMake
cmake_minimum_required(VERSION 3.10)

# Назва нашого проєкту
project(TelemetryConverter CXX)

# Визначаємо версію стандарту C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Додаємо каталог з бібліотекою MAVLink як інкул-директорію
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/
)

# Визначаємо вихідні файли для бібліотеки
set(CONVERTER_SOURCE src/msp_converter.cpp)

# Створюємо перший виконуваний файл для роботи з файлами
add_executable(TelemetryConverter_File src/main.cpp ${CONVERTER_SOURCE})

# Створюємо другий виконуваний файл для роботи з послідовним портом
add_executable(TelemetryConverter_Serial src/serial_main.cpp ${CONVERTER_SOURCE})