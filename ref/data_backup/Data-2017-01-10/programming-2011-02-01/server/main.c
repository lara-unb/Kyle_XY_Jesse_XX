/*******************************************************************************
* main.c: Modulo principal do projeto do servidor.
* Observacoes:
* 	- 
*******************************************************************************/
/*! \file main.c
* \brief Arquivo principal. */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/io.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>

#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>

#include "robotcommondefs.h"
#include "robot.h"
#include "robotserver.h"
#include "keyboard.h"

// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

// Cabecalhos especificos do modulo:
void catch_signal(int sig);

// Variáveis do modulo:
volatile int flag_quit = 0;


int main (int argc, char *argv[])
{       
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	printf("\n\n\n\n***************************************************************");
	printf("\n**** Mobile Mapipulator Server Process");
	printf("\n***************************************************************\n");

	// robot module:
	printf("\n*** Iniciando o modulo robot...");
	MAIN_MODULE_INIT(robot_init());

	// robotserver module:
	printf("\n*** Iniciando o modulo servidor...");
	MAIN_MODULE_INIT(robotserver_init());
	
	printf("\n*** Servidor pronto e processando requisições.");
	while(!flag_quit && !kbhit()){
		usleep(200000);
	}

	printf("\n*** Encerrando o modulo servidor...");
	MAIN_MODULE_CLOSE(robotserver_close());
	
	printf("\n*** Encerrando o modulo robot...");
	MAIN_MODULE_CLOSE(robot_close());
	
	printf("\n\n");
    return 1;
}

void catch_signal(int sig)
{
}
