# Визначаємо мінімальну версію CMake
cmake_minimum_required(VERSION 3.10)

# Назва нашого проєкту
project(TelemetryConverter CXX)

# Визначаємо версію стандарту C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Прапорці компіляції: оптимізація та попередження (крос-платформено)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

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

# Компілер-специфічні прапорці (без прив'язки до ОС)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(TelemetryConverter_File PRIVATE -O2 -Wall -Wextra)
    target_compile_options(TelemetryConverter_Serial PRIVATE -O2 -Wall -Wextra)
elseif(MSVC)
    target_compile_options(TelemetryConverter_File PRIVATE /O2 /W4)
    target_compile_options(TelemetryConverter_Serial PRIVATE /O2 /W4)
endif()