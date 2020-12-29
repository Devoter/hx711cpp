cmake_minimum_required(VERSION 3.7)
project(hx711)
set(hx711_VERSION_MAJOR 2)
set(hx711_VERSION_MINOR 2)
set(hx711_VERSION_PATCH 1)

configure_file (
        "${PROJECT_SOURCE_DIR}/config.h.in"
        "${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set (warnings "-Wall -Wextra -Werror")
add_executable(${PROJECT_NAME} simple_kalman_filter.cpp string_to_double.cpp double_to_string.cpp hx711.cpp main.cpp)

find_package(Threads REQUIRED)
find_library(wiringPi_LIB wiringPi)

include_directories(${OPENSSL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${wiringPi_LIB} ${CMAKE_THREAD_LIBS_INIT})
