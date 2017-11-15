/*******************************************************************************
* main.c: Modulo principal do projeto do cliente.
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

#ifndef MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	#define MMROBOTCLIENT_COMPILE_FOR_XENOMAI  0
#endif

#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	#include <sys/mman.h>
	#include <native/task.h>
	#include <native/timer.h>
#endif

#include "gqueue.h"
#include "gmatlabdatafile.h"
#include "gdatalogger.h"
#include "keyboard.h"
#include "robotcommondefs.h"
#include "robotclient.h"
#include "timertask.h"

// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

// Cabecalhos especificos do modulo:
void catch_signal(int sig);
int timertask1(void);
int timertask2(void);

// Variáveis do modulo:
volatile int flag_quit = 0;
volatile int flag_quit_timertask1 = 0;
volatile int flag_quit_timertask2 = 0;
GDATALOGGER gDataLogger;

int timertask1(void)
{
	static float tshowmsg = 0.0;
	static int flag_firstexecution = 1;

	float t, ti, tf,time_robot, T_exec;
	servomovecommand_t servomovecommand;
	static int servo_pulse = 500;
	int status,flag_printmsg;
	static double value;

	// Na primeira execução, define para o datalogger as variáveis a serem salvas no arquivo de dados Matlab.
	if(flag_firstexecution){
		gDataLogger_DeclareVariable(&gDataLogger,"t1","s",1,1,1000);		// time
		gDataLogger_DeclareVariable(&gDataLogger,"T1_exec","s",1,1,1000);	// execution time
	}

	// Tempo da tarefa
	t = timertask_gettime();
	if(t > tshowmsg){
		flag_printmsg = 1;
		tshowmsg = t + 0.3;  
	} else{
		flag_printmsg = 0;
	}
	if(flag_printmsg) printf("\n *** Tarefa de controle: ");
	if(flag_printmsg) printf("\n t = %f s",t);
	
	// Leitura do tempo no robô:
	ti = timertask_gettime(); 
	status = robotclient_get_time(&time_robot); 
	tf = timertask_gettime(); 
	if(status){
		if(flag_printmsg) printf("\n robotclient_get_time executado com sucesso em %f ms",(tf-ti)*1e3);
	} else{
		if(flag_printmsg) printf("\n robotclient_get_time falhou");
	}

	// Envia comando para os servos:
	if((servo_pulse += 10) > 2900){
		servo_pulse = 100;
	}
	
	SERVOMOVECOMMAND_RESETMASK(&servomovecommand);
	
	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1500;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1;
	servomovecommand.pulse_speed_us_per_s[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 200;
	servomovecommand.pulse_speed_us_per_s_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_0] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1500;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1;
	servomovecommand.pulse_speed_us_per_s[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 200;
	servomovecommand.pulse_speed_us_per_s_mask[ROBOTPROTOCOL_SERVO_MANIPULADOR_1] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_LEFTWHEELS] = servo_pulse;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_LEFTWHEELS] = 1;

	servomovecommand.pulse_us[ROBOTPROTOCOL_SERVO_RIGHTWHEELS] = servo_pulse;
	servomovecommand.pulse_us_mask[ROBOTPROTOCOL_SERVO_RIGHTWHEELS] = 1;
	
	ti = timertask_gettime(); 
	status = robotclient_set_servos_move(&servomovecommand); 
	tf = timertask_gettime(); 
	if(status){
		if(flag_printmsg) printf("\n robotclient_set_servos_move executado com sucesso em %f ms",(tf-ti)*1e3);
	} else{
		if(flag_printmsg) printf("\n robotclient_set_servos_move falhou");
	}

	// Salvar variáveis de interesse no datalogger
	value = (double)t;
	gDataLogger_InsertVariable(&gDataLogger,"t1",&value);

	T_exec = timertask_gettime() - t;
	value = (double)T_exec;
	gDataLogger_InsertVariable(&gDataLogger,"T1_exec",&value);	// execution time

	// Atualiza estado de primeira execução.
	flag_firstexecution = 0;

	// Verifica se foi solicitado encerramento da tarefa. Caso sim, sinaliza isso.
	if(flag_quit_timertask1){
		flag_quit_timertask1 = 0;
		return 0;
	}
	return 1;
}

int timertask2(void)
{
	// Verifica se foi solicitado encerramento da tarefa. Caso sim, sinaliza isso.
	if(flag_quit_timertask2){
		flag_quit_timertask2 = 0;
		return 0;
	}
	return 1;
}

int main (int argc, char *argv[])
{       
	timertaskcontrol_t timertaskcontrol_task1;
	timertaskcontrol_t timertaskcontrol_task2;
	int i;
	char *pserveripaddress;
	
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	printf("\n\n\n\n***************************************************************");
	printf("\n**** Mobile Mapipulator Client Process");
	printf("\n***************************************************************\n");

	// gdatalogger module:
	printf("\n*** Iniciando o modulo gdatalogger...");
	MAIN_MODULE_INIT(gDataLogger_Init(&gDataLogger,"matlabdatafiles/gmatlabdatafile.mat",NULL));

	// timertask module:
	printf("\n*** Iniciando o modulo timertask...");
	MAIN_MODULE_INIT(timertask_init());
	
	// robotserver module:
	printf("\n*** Iniciando o modulo cliente...");
	if(argc==2){
		// endereço IP do servidor passado pelo usuário
		pserveripaddress = argv[1]; 
	}
	else{
		// endereço IP do localhost, ou seja, o servidor está rodando nessa mesma máquina.
		pserveripaddress = (char *)"127.0.0.1";
	}
	if(!robotclient_init(pserveripaddress)){
		printf("\n    Servidor não encontrado no endereço %s.",pserveripaddress);
		printf("\n*** Saindo do cliente...\n\n");
		return 0;
	}
	
	timertask_start (&timertaskcontrol_task1, timertask1, "task1", 20000, 0, 99);
	timertask_start (&timertaskcontrol_task2, timertask2, "task2", 50000, 0, 99);

	printf("\n*** Cliente pronto e executando.");
	i = 0;
	while(!flag_quit && !kbhit()){
		// Dorme por 20ms
		usleep(20000);
		// Procedimentos de gerenciamento do datalogger
		gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
		if(++i>10){
			gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
			i = 0;
		}
	}

	printf("\n*** Encerrando as tarefas periódicas...");
	printf("\n    Encerrando tarefa 1 ... ");
	for(flag_quit_timertask1 = 1, i = 100 ; flag_quit_timertask1; --i){
		usleep(10000);
		if(i <= 0){
			printf(" tarefa nao responde. Forçando parada ...");
			break;
		}
	}
	timertask_kill (&timertaskcontrol_task1);
	printf(" tarefa encerrada.");

	printf("\n    Encerrando tarefa 2 ... ");
	for(flag_quit_timertask2 = 1, i = 100 ; flag_quit_timertask2; --i){
		usleep(10000);
		if(i <= 0){
			printf(" tarefa nao responde. Forçando parada ...");
			break;
		}
	}
	timertask_kill (&timertaskcontrol_task2);
	printf(" tarefa encerrada.");

	printf("\n*** Encerrando o modulo cliente...");
	MAIN_MODULE_CLOSE(robotclient_close());
	
	printf("\n*** Encerrando o modulo datalogger...");
	MAIN_MODULE_CLOSE(gDataLogger_Close(&gDataLogger));

	printf("\n\n");
    return 1;
}

void catch_signal(int sig)
{
}
