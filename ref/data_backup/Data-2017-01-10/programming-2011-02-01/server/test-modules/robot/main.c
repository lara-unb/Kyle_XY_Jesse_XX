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
int mode1_handler(void);
int mode2_handler(void);
int mode3_handler(void);
int mode4_handler(void);

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

// Variáveis do modulo:

int main (int argc, char *argv[])
{       
	int flag_quit = 0;
	int mode = 0;
	
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	printf("\n\n\n\n***************************************************************");
	printf("\n**** Rotinas de teste do modulo robot");
	printf("\n***************************************************************\n");
	printf("\n");

	// robot module:
	printf("\n*** Iniciando o modulo robot...");
	MAIN_MODULE_INIT(robot_init());
	MAIN_MODULE_INIT(sensoray526_init());
	
	printf("\n*** Escolha uma opção:");
	printf("\n   (1): testar robot_set_servos_move");
	printf("\n\n   Opção: ");
	mode = getch() - '0';
	printf("%i",mode);

	printf("\n\n*** Executando modo %i",mode);
	
	while(!flag_quit && !kbhit()){
		switch(mode){
			case 1: if(!mode1_handler()) flag_quit = 1; break;
			case 2: if(!mode2_handler()) flag_quit = 1; break;
			case 3: if(!mode3_handler()) flag_quit = 1; break;
			case 4: if(!mode4_handler()) flag_quit = 1; break;
			default:
				printf("\n Modo não implementado"); flag_quit = 1;
		}
	}

	printf("\n*** Encerrando o modulo robot...");
	MAIN_MODULE_CLOSE(robot_close());
	MAIN_MODULE_INIT(sensoray526_close());
	
	
	printf("\n\n");
	fflush(stdout); // mostra todos printfs pendentes.
    return 1;
}

void catch_signal(int sig)
{
}

int mode1_handler(void)
{
	//static int flag_firstexecution = 1;

	float texec;
	servomovecommand_t servomovecommand;
	int status;
		
	// Sleep
	usleep(200000);

	// Envia comando para os servos:
	SERVOMOVECOMMAND_RESETMASK(&servomovecommand);
	
	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1500;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1;
	servomovecommand.pulse_speed_us_per_s[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 200;
	servomovecommand.pulse_speed_us_per_s_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1500;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1;
	servomovecommand.pulse_speed_us_per_s[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 200;
	servomovecommand.pulse_speed_us_per_s_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_LEFTWHEELS] = 1800;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_LEFTWHEELS] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_RIGHTWHEELS] = 1800;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_RIGHTWHEELS] = 1;
	
	tic(); 
	status = robot_set_servos_move(&servomovecommand); 
	texec = toc(); 
	if(status){
		printf("\n robot_set_servos_move executado com sucesso em %f ms",texec*1e3);
	} else{
		printf("\n robot_set_servos_move falhou");
	}
	
	return status;
}

int mode2_handler(void)
{
	printf("\n Modo não implementado"); return 0;
	
	// Sleep
	usleep(1000000);
	
	return 1;
}

int mode3_handler(void)
{
	printf("\n Modo não implementado"); return 0;
	
	// Sleep
	usleep(1000000);
	
	return 1;
}

int mode4_handler(void)
{
	printf("\n Modo não implementado"); return 0;
	
	// Sleep
	usleep(1000000);
	
	return 1;
}
