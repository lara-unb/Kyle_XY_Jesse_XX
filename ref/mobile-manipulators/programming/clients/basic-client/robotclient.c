/*****************************************************************************
*** Arquivo: robotclient.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file robotclient.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */
#include <string.h>

// Cabecalhos especificos do modulo:
#include "socketmsg-x86.h"
#include "robotcommondefs.h"
#include "robotclient.h"

// Definicoes internas:

// Prototipos internos:

// Variaveis do modulo:
socketmsg_t SocketMsgStruct_DataPort;
messsage_t  Message_DataPort;

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int robotclient_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int robotclient_init(char *serveripnumber)
{
	// Tenta conexão com o servidor:
	if(socketmsg_client_init(serveripnumber, SOCKETMSG_SERVERADDRESSMODE_IP, ROBOTSERVER_PORTNUMBER_DATA, &SocketMsgStruct_DataPort, (2 * 1280 * 960 * 3 + 100), 0)==0){
		return 0;
	}

	// Inicializa threads.

	// Retorna 
	return 1; 
}                      

/*! \fn int robotclient_close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/
int robotclient_close(void)
{
	// Procedimentos de encerramento
	if(socketmsg_close(&SocketMsgStruct_DataPort)==0){
		return 0; 
	}
	
	// Retorna 
   	return 1; 
}                      


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
/*! \fn int robotclient_gettime_ms(double *ptimems)
* \brief Solicita o tempo atual no robô
* \param none
* \return If success, 1. Otherwise, 0.
*/
int robotclient_get_time(float *ptime)
{
	// Argumentos da chamada:
	Message_DataPort.Header = ROBOTPROTOCOL_MSGHEADER_GET_TIME;
	Message_DataPort.NData = 0;

	// Envia a mensagem e aguarda resposta:
	socketmsg_send_message(&SocketMsgStruct_DataPort, &Message_DataPort);
	socketmsg_wait_message(&SocketMsgStruct_DataPort, &Message_DataPort);

	// Processa resposta:
	if(Message_DataPort.Header == ROBOTPROTOCOL_MSGHEADER_ACK){
		memcpy((unsigned char *)ptime, Message_DataPort.pData[0], Message_DataPort.DataSizes[0]);
		return 1;
	}
	if(Message_DataPort.Header == ROBOTPROTOCOL_MSGHEADER_ERROR){
		return 0;
	}
	return 0;
}

/*! \fn int robotclient_gettime_ms(double *ptimems)
* \brief Solicita o tempo atual no robô
* \param none
* \return If success, 1. Otherwise, 0.
*/
int robotclient_set_servos_move(servomovecommand_t *pservomovecommand)
{
	// Argumentos da chamada:
	Message_DataPort.Header = ROBOTPROTOCOL_MSGHEADER_SET_SERVOSMOVE;
	Message_DataPort.NData = 1;
	
	Message_DataPort.pData[0] = (unsigned char *) pservomovecommand;
	Message_DataPort.DataSizes[0] = sizeof(servomovecommand_t);

	// Envia a mensagem e aguarda resposta:
	socketmsg_send_message(&SocketMsgStruct_DataPort, &Message_DataPort);
	socketmsg_wait_message(&SocketMsgStruct_DataPort, &Message_DataPort);

	// Processa resposta:
	if(Message_DataPort.Header == ROBOTPROTOCOL_MSGHEADER_ACK){
		return 1;
	}
	if(Message_DataPort.Header == ROBOTPROTOCOL_MSGHEADER_ERROR){
		return 0;
	}
	return 0;
}

/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

