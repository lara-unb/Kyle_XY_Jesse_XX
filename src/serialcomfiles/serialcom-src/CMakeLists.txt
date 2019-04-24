cmake_minimum_required(VERSION 2.8)
project(SerialCom)

include_directories(include)
add_definitions(-g -O3 -Wall)

set(SOURCES src/serialcom.c)

add_library(serialcomlib SHARED ${SOURCES})
