/*****************************************************************************
*** Arquivo: robotserver.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file robotserver.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */
#include <sys/time.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

/*
#include <cv.h>				
#include <cvaux.h>			
#include <cxcore.h>		
*/
// Cabecalhos especificos do modulo:
#include "socketmsg-x86.h"
#include "robotcommondefs.h"
#include "robot.h"
#include "robotserver.h"

// Definicoes internas:
#define ROBOTSERVER_TASK_DATA_PRIORITY		20
#define ROBOTSERVER_TASK_DATA_STACK			4096
#define ROBOTSERVER_TASK_DATA_PERIOD_NS		1000000 /* periodo em nanosegundos (1ms) */

// Prototipos internos:
void robotserver_data_task_handler(void *arg);

// Variaveis do modulo:
RT_TASK robotserver_camera_task;
RT_TASK robotserver_data_task;
static char quittask = 0;

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int robotserver_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int robotserver_init(void)
{
	int status;
	
	// Cria tarefas
	status = rt_task_create(&robotserver_data_task, "robotserver_data_task", ROBOTSERVER_TASK_DATA_STACK, ROBOTSERVER_TASK_DATA_PRIORITY, T_JOINABLE);
	if (status != 0) {
		printf("    Criacao do thread 'robotserver_data_task' falhou: ");
		if(status == (-ENOMEM)) printf("-ENOMEM\n");
		if(status == (-EEXIST)) printf("-EEXIST\n");
		if(status == (-EPERM)) printf("-EPERM\n");
		status = rt_task_delete(&robotserver_data_task);
		if (status != 0) {
			printf("    Falha na tentativa de deletar 'robotserver_data_task'.\n");
		}
		return 0;
	}
	status = rt_task_start(&robotserver_data_task, &robotserver_data_task_handler, NULL);
	if (status != 0) {
		printf("    Lancamento do thread 'robotserver_data_task' falhou.\n");
		return 0;
	}

	// Retorna 
	return 1; 
}                      

/*! \fn int robotserver_close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/

int robotserver_close(void)
{
	// Procedimentos de encerramento
	quittask = 1;	
	rt_task_delete(&robotserver_camera_task);	
	rt_task_delete(&robotserver_data_task);	
		
	// Retorna 
    return 1; 
}                      


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/


/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

/*! \fn void robotserver_data_task_handler(void *arg)
* \brief Gerenciador de comunicação de dados
* \param none
* \return none
*/
void robotserver_data_task_handler(void *arg)
{
	messsage_t Message;
	socketmsg_t SocketMsgStruct;
	float varfloat[10];

	// Cria o socket
	if( socketmsg_server_init(ROBOTSERVER_PORTNUMBER_DATA, &SocketMsgStruct, ROBOTSERVER_MAXMSGSIZE, 0)==0){
		printf("\n*** Erro em socketmsg_server_init()");
		socketmsg_close(&SocketMsgStruct);
		return;
	}

	rt_task_set_periodic(NULL, TM_NOW, ROBOTSERVER_TASK_DATA_PERIOD_NS);

	// A funcao periodica eh executada dentro desse while. 
	while(!quittask) {
		// Aguarda proximo instante de execucao
		rt_task_wait_period(NULL);

		// Aguarda conexão
		while(!socketmsg_server_acceptconnection(&SocketMsgStruct)) {
			// Aguarda proximo instante de execucao
			rt_task_wait_period(NULL);
			if(quittask){
				if(socketmsg_close(&SocketMsgStruct)==0){
					printf(" Erro em socketmsg_close()"); 
				}
				return;
			}
		}
		
		// Processamento da conexão
//		printf("\n data_task_handler: Processando conexao...");
		while((socketmsg_wait_message(&SocketMsgStruct, &Message)==1) && (!quittask)) {
//			printf("\n data_task_handler: Mensagem recebida: Message.Header = %X ,  Message.NData = %i",Message.Header,Message.NData); 
			switch(Message.Header){
			case ROBOTPROTOCOL_MSGHEADER_SET_SERVOSMOVE:
				if(Message.NData == 1){
					if(robot_set_servos_move(((servomovecommand_t *)(Message.pData[0])))){
						// resposta: ACK
						Message.Header = ROBOTPROTOCOL_MSGHEADER_ACK;
						Message.NData = 0;
					} else{
						printf(" robotserver_data_task_handler: robot_set_servos_move retornou com erro (mensagem ROBOTPROTOCOL_MSGHEADER_SET_SERVOSMOVE)"); 
						// resposta: ERROR
						Message.Header = ROBOTPROTOCOL_MSGHEADER_ERROR;
						Message.NData = 0;
					}
				}
				else{
					printf(" robotserver_data_task_handler: Erro no tamanho de dados da mensagem ROBOTPROTOCOL_MSGHEADER_SET_SERVOSMOVE"); 
					// resposta: ERROR
					Message.Header = ROBOTPROTOCOL_MSGHEADER_ERROR;
					Message.NData = 0;
				}
				break;
			case ROBOTPROTOCOL_MSGHEADER_GET_TIME:
				if(Message.NData == 0){
					if(!robot_get_time(&varfloat[0])){
						printf(" robotserver_data_task_handler: Erro em robot_get_time()"); 
						// resposta: ERROR
						Message.Header = ROBOTPROTOCOL_MSGHEADER_ERROR;
						Message.NData = 0;
					} 
					else{
						// resposta: ACK + varfloat[0]
						Message.Header = ROBOTPROTOCOL_MSGHEADER_ACK;
						Message.pData[0] = (unsigned char *)&varfloat[0];
						Message.DataSizes[0] = sizeof(float);
						Message.NData = 1;
					}
				}
				else{
					printf(" robotserver_data_task_handler: Erro no tamanho de dados da mensagem ROBOTPROTOCOL_MSGHEADER_GET_TIME"); 
					// resposta: ERROR
					Message.Header = ROBOTPROTOCOL_MSGHEADER_ERROR;
					Message.NData = 0;
				}
				break;
			default: // funcao nao conhecida. reportar erro.
					// resposta: ERROR
					Message.Header = ROBOTPROTOCOL_MSGHEADER_ERROR;
					Message.NData = 0;
				break;
			}
			socketmsg_send_message(&SocketMsgStruct, &Message);

			// Aguarda proximo instante de execucao
			rt_task_wait_period(NULL);
		}

		// Fecha conexão
		close(SocketMsgStruct.clientsocketfd);
	}
	
	// Destroi o socket
	if(socketmsg_close(&SocketMsgStruct)==0){
		printf(" Erro em socketmsg_close()"); 
	}
}



