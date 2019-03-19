#include <stdio.h>
#include <unistd.h>
#include "serialcom.h"

int main(){
    SERIALPORTCONFIG serialPortConfig;
    int err;
    char* data= "#0 P1600\r\n";

    // Init the serial port
    if((err = serialcom_init(&serialPortConfig, 1, "/dev/ttyS0", 115200)) != SERIALCOM_SUCCESS)
    {
        return err;
    }
	printf("---------------------------------\n");
    // Receive byte
    //if((err = serialcom_receivebyte(&serialPortConfig, data, 1e4)) != SERIALCOM_SUCCESS)
    //{
    //    return err;
    //}

	printf("---------------------------------\n");
    // Send the same byte
    for (int i=0; i<strlen(data); i++)
    {
	serialcom_sendbyte(&serialPortConfig, (unsigned char*) &data[i]);
	usleep(5);
    }
    sleep(3);

    printf("---------------------------------\n");
    // Close the serial port
    if((err = serialcom_close(&serialPortConfig)) != SERIALCOM_SUCCESS)
    {
        return err;
    }

    return 0;
}
