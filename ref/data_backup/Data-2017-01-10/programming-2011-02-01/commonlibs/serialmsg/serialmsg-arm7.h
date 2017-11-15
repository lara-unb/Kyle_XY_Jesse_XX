/*******************************************************************************
* \file serialmsg-arm7.h
* \brief  Implementa modulo basico de comunicação cliente-servidor usando porta serial.
// Observaçoes:
//    - Inspirado no projeto Carcarah
*******************************************************************************/

#ifndef SERIALMSG_H
#define SERIALMSG_H

#include "lprotocol.h"
#include "serial-sam7.h"

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
#define SERIALMSG_MSGHANDLERSTATUS_ACK									0
#define SERIALMSG_MSGHANDLERSTATUS_ERROR_WRONG_NUMBER_OF_ARGUMENTS		2

#define SERIALMSG_MAX_MESSAGE_DATA_FIELDS	10

#define SERIALMSG_MODE_SERVER				3
#define SERIALMSG_MODE_CLIENT				4

typedef int (*pserialmsgfunctionhandler_t)(lprotocoldata_t*, lprotocoldatagram_t*);

typedef struct{
//	pthread_mutex_t mutex;
	pserialmsgfunctionhandler_t pmsgfunctionhandler[LPROCOTOL_MAX_FUNCTIONS];
	lprotocoldata_t protocoldata;
	lprotocoldatagram_t protocoldatagram;
	serialconfig_t serialportconfig;
	int mode;
	int	flagverbose;
} serialmsgcontrol_t;


// Prototipos de uso externo:
int serialmsg_server_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_init(int comportnumber, unsigned long int comportBPS, int flagverbose, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_close(serialmsgcontrol_t *pserialmsgcontrol);

int serialmsg_server_update(float time_ms, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_server_define_transaction_handler(unsigned char function_code, pserialmsgfunctionhandler_t pmsgfunctionhandler, serialmsgcontrol_t *pserialmsgcontrol);

int serialmsg_client_transaction_init(unsigned char function_code, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_insert_data(unsigned char *pdata, unsigned int data_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_retrieve_data(int slot, unsigned char **ppdata, unsigned int *pdata_size_in_bytes, serialmsgcontrol_t *pserialmsgcontrol);
int serialmsg_client_transaction_execute(serialmsgcontrol_t *pserialmsgcontrol);

#ifdef __cplusplus
}
#endif 

#endif // SERIALMSG_H
