# serialcom

The serialcom library provides accessing serial ports on LINUX systems with real-time RTAI compatibility. This library was thought to be compatible with a process with multiples threads and allows that multiples threads can access the same serial port each time. In that case, these functionalities are implemented to COM1, COM2, COM3, and COM4. Also, this library implements functions to access by semaphores, which allows the same serial port to be access by multiples threads per once.

# Quickstart

### Dependencies

* Only [pthreads](https://en.wikipedia.org/wiki/POSIX_Threads), that is usually installed within any Linux system.

### Usage

#### Link

A CMake code is provided in hopes of help the link process.
To link this library against your project use the code below in your CMakeLists.txt file.

```bash
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
include_directories(${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/include)
# message(${install_dir})

add_executable( <YOUR_EXECUTABLE> <YOUR_CODE>.cpp )
add_dependencies(<YOUR_EXECUTABLE> serialcom)
target_link_libraries( <YOUR_EXECUTABLE>
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}serialcomlib.so
    -pthread
)
```
#### API

```c
#include <serialcom.h>

int main()
{
    SERIALPORTCONFIG serialPortConfig;
    char* data;

    // Init the serial port
    if((err = serialcom_init(&serialPortConfig, 1, "/dev/ttyUSB0", 9600)) != SERIALCOM_SUCCESS)
    {
        return err;
    }

    // Receive byte
    if((err = serialcom_receivebyte(&serialPortConfig, &data, 1e4)) != SERIALCOM_SUCCESS)
    {
        return err;
    }

    // Send the same byte
    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &data);
    usleep(5);

    // Close the serial port
    if((err = serialcom_close(&serialPortConfig)) != SERIALCOM_SUCCESS)
    {
        return err;
    }

    return 0;
}
```

# Documentation

Documentation (and further explanations) for the code is available in the source code files.
