//#include <string>
//#include <cstdlib>

// Cabecalho especificos do modulo:
#include "SSC.h"
#include "serialcom.h"
const char* cr = "\r\n";

int sendCommand(const char* data)
{
    SERIALPORTCONFIG serialPortConfig;
    int err;
    char ssc[] = "/dev/ttyS0";

    // Init the serial port
    if((err = serialcom_init(&serialPortConfig, 1, ssc, 115200)) != SERIALCOM_SUCCESS)
    {
        return err;
    }
    // Send command
    for (int i=0; i<strlen(data); i++)
    {
	serialcom_sendbyte(&serialPortConfig, (unsigned char*) &data[i]);
	usleep(5);
    }
    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &cr[0]);
	usleep(5);
    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &cr[1]);
	usleep(5);
     // Close the serial port
    if((err = serialcom_close(&serialPortConfig)) != SERIALCOM_SUCCESS)
    {
        return err;
    }
}