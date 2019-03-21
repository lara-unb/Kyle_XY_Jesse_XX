// Bibliotecas
#include <stdio.h>
#include <unistd.h>
#include "serialcom.h"
#include <string>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/io.h>
#include <sys/time.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <sys/select.h>
#include "sensoray526.h"

// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

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

struct{
	struct timeval time;
	struct timeval timereset;
} tictocctrl;

void tic(void)
{
	gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
	gettimeofday(&tictocctrl.time, NULL);
	return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec)*1e-6);
}

int kbhit(void)
{
  struct timeval tv;
  fd_set read_fd;

  /* Do not wait at all, not even a microsecond */
  tv.tv_sec=0;
  tv.tv_usec=0;

  /* Must be done first to initialize read_fd */
  FD_ZERO(&read_fd);

  /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
  FD_SET(0,&read_fd);

  /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
  if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */

  /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */
  
if(FD_ISSET(0,&read_fd))
    /* Character pending on stdin */
    return 1;

  /* no characters were pending */
  return 0;
}

void catch_signal(int sig)
{
}

int readEncoder(void)
{
    float texec;
	unsigned char counter=0;
	//Configura encoder 0
	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);
	sensoray526_configure_encoder(2);
	sensoray526_configure_encoder(3);
	sensoray526_reset_counter(0);
	sensoray526_reset_counter(1);
	sensoray526_reset_counter(2);
	sensoray526_reset_counter(3);
	tic(); 
	for(counter=0;counter<10;counter++)
	{
		//Le o encoder 0
		printf("\n Encoder 0: %X (%d)",sensoray526_read_counter(0),sensoray526_read_counter(0));
		printf("\n Encoder 1: %X (%d)",sensoray526_read_counter(1),sensoray526_read_counter(1));
		printf("\n Encoder 2: %X (%d)",sensoray526_read_counter(2),sensoray526_read_counter(2));
		printf("\n Encoder 3: %X (%d)",sensoray526_read_counter(3),sensoray526_read_counter(3));
	
		// Sleep
		usleep(100000);
	}
	texec = toc(); //us
    printf("Read encoder took %f seconds to execute \n", texec*1e6);
	return 1;
}

int angVelocity(void)
{
    float texec;
	unsigned char counter=0;
    int n0 = 0;
    int n1 = 0;
    double w;
	//Configura encoder 0
	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);
	sensoray526_reset_counter(0);
	sensoray526_reset_counter(1);
	tic(); 

    // Sleep
	usleep(100000);
        
    n0 = sensoray526_read_counter(0) - n0; //n of pulses encoder 0
    n1 = sensoray526_read_counter(1) - n0; //n of pulses encoder 1

    texec = toc();
    w = (2 * 3.14159275 * n0) / (100 * texec); 
    printf("\n Speed: %f rad/s", w);
	return 1;
}

int main(){

    int flag_quit = 0;
	int mode = 0;
     std::string command;

    signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
    
    //run 
    command = "#0 P1600 #1 P1600";
    sendCommand(command.c_str());
    angVelocity();
    angVelocity();
    angVelocity();

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	// robot module:
	printf("\n*** Iniciando o modulo sensoray526...");
	MAIN_MODULE_INIT(sensoray526_init());
	
	while(!flag_quit && !kbhit()){
		readEncoder();
        flag_quit = 1; 
	}
    command = "#0 P1500 #1 P1500";
    sendCommand(command.c_str());
    sleep(1);

	printf("\n*** Encerrando o modulo robot...");
	MAIN_MODULE_CLOSE(sensoray526_close());
	
	printf("\n\n");
	fflush(stdout); // mostra todos printfs pendentes.
    return 1;
}
