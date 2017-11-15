/*******************************************************************************
* \file serialmsg-x86-linux.h
* \brief  Implementa modulo basico de comunicação cliente-servidor usando porta serial.
// Observaçoes:
//    - Inspirado no projeto Carcarah
*******************************************************************************/

#ifndef SERIALMSG_H
#define SERIALMSG_H

#include "lprotocol.h"
#include "serialcom.h"

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
#define SERIALMSG_MAX_MESSAGE_DATA_FIELDS	10

#define SERIALMSG_MODE_SERVER				3
#define SERIALMSG_MODE_CLIENT				4

#define SERIALMSG_TRANSACTIONPARAMETER_MAX_WAIT_INTERBYTE_TIME_US 	5000


typedef struct{
	pthread_mutex_t mutex;
	lprotocoldata_t protocoldata;
	lprotocoldatagram_t protocoldatagram;
	SERIALPORTCONFIG serialportconfig;
	int mode;
	int	flagverbose;
} serialmsgcontrol_t;


// Prototipos de uso externo:
#if SERIALCOM_USE_DIRECT_IO_MODE
	int serialmsg_server_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
	int serialmsg_client_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
#else
	int serialmsg_server_init(int comportnumber, char *pcomdevice, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
	int serialmsg_client_init(int comportnumber, char *pcomdevice, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
#endif
int serialmsg_close(serialmsgcontrol_t *pserialmsgcontrol);

int serialmsg_server_transaction_handling_update(serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_server_define_transaction_handler(unsigned char function_code, serialmsgcontrol_t *pserialmsgcontrol);

int serialmsg_client_transaction_init(unsigned char function_code, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_insert_data(unsigned char *pdata, unsigned int data_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_retrieve_data(int slot, unsigned char **ppdata, unsigned int *pdata_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_execute(serialmsgcontrol_t *pserialmsgcontrol);

#ifdef __cplusplus
}
#endif 

#endif // SERIALMSG_H
