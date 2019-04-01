cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(Basics_motion)

include(ExternalProject)
ExternalProject_Add(serialcom
    GIT_REPOSITORY https://github.com/Gastd/serialcom
    GIT_TAG master
    # PREFIX ${CMAKE_CURRENT_BINARY_DIR}/serialcom
    SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/build"
    # CONFIGURE_COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${CMAKE_BINARY_DIR}/serialcom-src/"
    BUILD_COMMAND "${CMAKE_COMMAND}" --build .
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)
# ExternalProject_Get_Property(serialcom install_dir)
ExternalProject_Get_Property(serialcom binary_dir)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/include
		include)

# message(${install_dir})

add_library(sensoray526 SHARED src/sensoray526.c include/sensoray526.h)

install(TARGETS sensoray526 DESTINATION lib)
install(FILES src/sensoray526.h DESTINATION include)

add_executable(serialTest src/serialTest.cpp)
add_dependencies(serialTest serialcom)
target_link_libraries( serialTest
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}serialcomlib.so
    -pthread
    sensoray526
)
