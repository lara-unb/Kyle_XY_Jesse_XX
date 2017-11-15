/*****************************************************************************
*** Arquivo: robot.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 12-01-2011: criacao
*****************************************************************************/
/*! \file robot.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */
#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

// Cabecalhos especificos do modulo:
#include "serialcom.h" 
#include "robotcommondefs.h"
#include "sensoray526.h"
#include "robot.h"

// Definicoes internas:
#define ROBOT_TASK_SERVOS_PRIORITY		99
#define ROBOT_TASK_SERVOS_STACK			4096
#define ROBOT_TASK_SERVOS_PERIOD_NS		1000000 /* periodo em nanosegundos (1ms) */

// Prototipos internos:
void robot_servos_task_handler(void *arg);

// Variaveis do modulo:
static struct timeval timereset;
static SERIALPORTCONFIG ssc32serialportconfig; 
RT_TASK robot_servos_task;
static char robot_quittask = 0;

volatile int servos_pulse_us[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
volatile int servos_pulse_us_max[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
volatile int servos_pulse_us_min[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int robot_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return 1 Sucesso
* \return 0 Falha.
*/
int robot_init(void)
{
	int status,i;
	
	// Variaveis:
	gettimeofday(&timereset, NULL);
	for(i=0;i<ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS;++i){
		servos_pulse_us[i] = 1500;
		servos_pulse_us_max[i] = 2500;
		servos_pulse_us_min[i] = 500;
	}
 
	// Porta serial COM1 à qual está conectada a placa SSC-32:
	if ( (status=serialcom_init(&ssc32serialportconfig, 1, 115200) ==  SERIALCOM_SUCCESS)){
		printf("\n %s: porta seriam COM1 iniciada com sucesso",__FUNCTION__);
	} else {
		printf("\n %s: porta seriam COM1 com erro de código %i",__FUNCTION__,status);
		return 0;
	}

	// Cria tarefas
	status = rt_task_create(&robot_servos_task, "robot_servos_task", ROBOT_TASK_SERVOS_STACK, ROBOT_TASK_SERVOS_PRIORITY, T_JOINABLE);
	if (status != 0) {
		printf("    Criacao do thread 'robot_servos_task' falhou: ");
		if(status == (-ENOMEM)) printf("-ENOMEM\n");
		if(status == (-EEXIST)) printf("-EEXIST\n");
		if(status == (-EPERM)) printf("-EPERM\n");
		status = rt_task_delete(&robot_servos_task);
		if (status != 0) {
			printf("    Falha na tentativa de deletar 'robot_servos_task'.\n");
		}
		return 0;
	}
	status = rt_task_start(&robot_servos_task, &robot_servos_task_handler, NULL);
	if (status != 0) {
		printf("    Lancamento do thread 'robot_servos_task' falhou.\n");
		return 0;
	}

	// Retorna 
	return 1; 
}                      

/*! \fn int robot_close(void)
* \brief Funcao de encerramento
* \param none
* \return 1 Sucesso
* \return 0 Falha.
*/

int robot_close(void)
{
	// Porta serial COM1 à qual está conectada a placa SSC-32:
	serialcom_close(&ssc32serialportconfig);

	// Procedimentos de encerramento
	robot_quittask = 1;	
	rt_task_delete(&robot_servos_task);	
	
	// Retorna 
   	return 1; 
}                      


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
/*! \fn int robot_close(void)
* \brief Funcao de encerramento
* \param none
* \return 1 Sucesso
* \return 0 Falha.
*/
int robot_get_time(float *ptime)
{
	struct timeval time;

	gettimeofday(&time, NULL);
	*ptime = ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec)*1e-6);
	
	return 1;
}

/*! \fn robot_set_servos_move(servomovecommand_t *pservomovecommand)
* \brief Funcao de encerramento
* \param pservomovecommand ponteiro para estrutura servomovecommand_t
* \return 1 Sucesso
* \return 0 Falha.
*/
int robot_set_servos_move(servomovecommand_t *pservomovecommand)
{
	int i;
	
	for(i=0;i<ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS;++i){
		if(pservomovecommand->pulse_us_mask[i]){
			servos_pulse_us[i] = pservomovecommand->pulse_us[i];
			if(pservomovecommand->pulse_speed_us_per_s_mask[i]){
			}
			if(pservomovecommand->pulse_timeformove_ms_mask[i]){
			}
		}
	}

	return 1;
}

/*! \fn robot_set_servos_move(servomovecommand_t *pservomovecommand)
* \brief Funcao de encerramento
* \param pservomovecommand ponteiro para estrutura servomovecommand_t
* \return 1 Sucesso
* \return 0 Falha.
*/
/*
int robot_set_servos_move(servomovecommand_t *pservomovecommand)
{
	int i;
	char ssc32msg[200];
	
	// Converter a estrutura de comando em mensagem para a placa ssc-32:
	ssc32msg[0] = '\0'; // inicia vazio.
	for(i=0;i<ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS;++i){
		if(pservomovecommand->pulse_us_mask[i]){
			if(strlen(ssc32msg)!=0){
				sprintf(ssc32msg,"%s ",ssc32msg);
			}
			sprintf(ssc32msg,"%s#%i P%i",ssc32msg,i,pservomovecommand->pulse_us[i]);
			if(pservomovecommand->pulse_speed_us_per_s_mask[i]){
				sprintf(ssc32msg,"%s S%i",ssc32msg,pservomovecommand->pulse_speed_us_per_s[i]);
			}
			if(pservomovecommand->pulse_timeformove_ms_mask[i]){
				sprintf(ssc32msg,"%s T%i",ssc32msg,pservomovecommand->pulse_timeformove_ms[i]);
			}
		}
	}
	sprintf(ssc32msg,"%s%c",ssc32msg,13); // carriage return

//	printf("\n *** robot_set_servos_move msg: %s",ssc32msg);
	
	// Enviar comando:
	serialcom_semwait(&ssc32serialportconfig); // reserva a porta serial para a thread que chama essa função
	for(i=0;i<strlen(ssc32msg);++i){
		serialcom_sendbyte(&ssc32serialportconfig,(unsigned char*)(&ssc32msg[i]));
	}
	serialcom_semsignal(&ssc32serialportconfig); // liberar a porta serial para ser usada por outra thread

	return 1;
}*/

/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

void robot_servos_task_handler(void *arg)
{
	int channel = 0;
	RTIME width;

	// A funcao periodica eh executada dentro desse while. 
	while(!robot_quittask) {
		if(++channel>=8) channel = 0;

		sensoray526_set_dio((1<<channel), 0xFF);
		if(servos_pulse_us[channel] > servos_pulse_us_max[channel])
			servos_pulse_us[channel] = servos_pulse_us_max[channel];
		if(servos_pulse_us[channel] < servos_pulse_us_min[channel])
			servos_pulse_us[channel] = servos_pulse_us_min[channel];
		width = servos_pulse_us[channel] * 1000;

		rt_task_sleep(width);
	}
}
