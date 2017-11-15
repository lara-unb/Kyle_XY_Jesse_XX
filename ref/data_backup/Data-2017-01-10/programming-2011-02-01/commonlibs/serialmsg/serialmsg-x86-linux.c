/*******************************************************************************
* \file serialmsg-x86-linux.c
* \brief  Implementa modulo basico de comunicação cliente-servidor usando porta serial.
// Observaçoes:
//    - Inspirado no projeto Carcarah
*******************************************************************************/

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */
#include <pthread.h>
#include <errno.h>

// Cabecalhos especificos do modulo:
#include "serialmsg-x86-linux.h"

// Definicoes internas:

// Prototipos internos:

// Variaveis do modulo:

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/
#if SERIALCOM_USE_DIRECT_IO_MODE
	int serialmsg_server_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol)
	{
		pserialmsgcontrol->mode = SERIALMSG_MODE_SERVER;
		pserialmsgcontrol->flagverbose = flagverbose;
		
		lprotocol_init();

		if(serialcom_init(&pserialmsgcontrol->serialportconfig, comportnumber, comportBPS)!=SERIALCOM_SUCCESS){
			return 0;
		}

		return 1;
	}

	int serialmsg_client_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol)
	{
		pserialmsgcontrol->mode = SERIALMSG_MODE_CLIENT;
		pserialmsgcontrol->flagverbose = flagverbose;

		lprotocol_init();

		if(serialcom_init(&pserialmsgcontrol->serialportconfig, comportnumber, comportBPS)!=SERIALCOM_SUCCESS){
			return 0;
		}

		return 1;
	}

#else
	int serialmsg_server_init(int comportnumber, char *pcomdevice, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol)
	{
		pserialmsgcontrol->mode = SERIALMSG_MODE_SERVER;
		pserialmsgcontrol->flagverbose = flagverbose;
		
		lprotocol_init();

		if(serialcom_init(&pserialmsgcontrol->serialportconfig, comportnumber, pcomdevice, comportBPS)!=SERIALCOM_SUCCESS){
			return 0;
		}

		return 1;
	}

	int serialmsg_client_init(int comportnumber, char *pcomdevice, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol)
	{
		pserialmsgcontrol->mode = SERIALMSG_MODE_CLIENT;
		pserialmsgcontrol->flagverbose = flagverbose;

		lprotocol_init();

		if(serialcom_init(&pserialmsgcontrol->serialportconfig, comportnumber, pcomdevice, comportBPS)!=SERIALCOM_SUCCESS){
			return 0;
		}

		return 1;
	}
#endif

int serialmsg_close(serialmsgcontrol_t *pserialmsgcontrol)
{
	serialcom_close(&pserialmsgcontrol->serialportconfig);

	lprotocol_close();

	return 0;
}

/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
int serialmsg_server_transaction_handling_update(serialmsgcontrol_t *pserialmsgcontrol)
{
	return 0; // não implementado
}

int serialmsg_server_define_transaction_handler(unsigned char function_code, serialmsgcontrol_t *pserialmsgcontrol)
{
	return 0; // não implementado
}

int serialmsg_client_transaction_init(unsigned char function_code, serialmsgcontrol_t *pserialmsgcontrol)
{
	lprotocol_datagram_encoder_init(function_code, &pserialmsgcontrol->protocoldata, &pserialmsgcontrol->protocoldatagram);

	return 1; 
}

int serialmsg_client_transaction_insert_data(unsigned char *pdata, unsigned int data_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol)
{
	lprotocol_datagram_encoder_insert_data(pdata, data_size_in_bytes, &pserialmsgcontrol->protocoldatagram);
	
	return 1; 
}

int serialmsg_client_transaction_retrieve_data(int slot, unsigned char **ppdata, unsigned int *pdata_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol)
{
	if((slot>=0) && (slot<pserialmsgcontrol->protocoldata.number_of_data)){
		*ppdata = pserialmsgcontrol->protocoldata.pdata[slot];
		*pdata_size_in_bytes = pserialmsgcontrol->protocoldata.data_size[slot];
	
		return 1; 
	}
	
	return 0;
}

int serialmsg_client_transaction_execute(serialmsgcontrol_t *pserialmsgcontrol)
{
	int n, statusdecode;
	unsigned char serial_data;
	
	if(pserialmsgcontrol->flagverbose) printf("\n serialmsg_client_transaction_execute: Mensagem enviada:  ");
	lprotocol_datagram_encoder_end(&pserialmsgcontrol->protocoldatagram);
	for(n=0;n<pserialmsgcontrol->protocoldatagram.datagram_size;++n){
		serial_data = pserialmsgcontrol->protocoldatagram.datagram[n];
		serialcom_sendbyte(&pserialmsgcontrol->serialportconfig, &serial_data);
		if(pserialmsgcontrol->flagverbose) printf(" %2X",serial_data);
	}

	if(pserialmsgcontrol->flagverbose) printf("\n serialmsg_client_transaction_execute: Mensagem recebida: ");
	lprotocol_datagram_decoder_reset(&pserialmsgcontrol->protocoldatagram);
	do{
		if(serialcom_receivebyte(&pserialmsgcontrol->serialportconfig, &serial_data, SERIALMSG_TRANSACTIONPARAMETER_MAX_WAIT_INTERBYTE_TIME_US)!=SERIALCOM_SUCCESS){
			if(pserialmsgcontrol->flagverbose) printf("\n serialmsg_client_transaction_execute: resposta não recebida.");
			return 0;
		}
		if(pserialmsgcontrol->flagverbose) printf(" %2X",serial_data);
		statusdecode = lprotocol_datagram_decoder_process_received_byte(serial_data, &pserialmsgcontrol->protocoldatagram);
		if(statusdecode == LPROTOCOL_ERROR_INVALID_DATAGRAM_CHECKSUM){
			if(pserialmsgcontrol->flagverbose) printf("\n serialmsg_client_transaction_execute: statusdecode == LPROTOCOL_ERROR_INVALID_DATAGRAM_CHECKSUM.");
			return 0;
		}
	}		
	while(statusdecode!=LPROTOCOL_DATAGRAM_RECEIVED);

//		lprotocol_datagram_print(&protocoldatagram);
	lprotocol_datagram_decoder_retrieve_data(&pserialmsgcontrol->protocoldata, &pserialmsgcontrol->protocoldatagram);
	if(pserialmsgcontrol->protocoldata.function_code==LPROCOTOL_FUNCTION_ACK){
		if(pserialmsgcontrol->flagverbose) printf("\n Transação reconhecida pelo servidor.");
	}
	else{
		if(pserialmsgcontrol->flagverbose) printf("\n Transação não reconhecida pelo servidor.");
		return 0;
	}

	return 1;
}

/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/
