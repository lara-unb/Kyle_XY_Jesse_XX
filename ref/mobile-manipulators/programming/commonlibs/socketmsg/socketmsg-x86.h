/*****************************************************************************
*** Projeto CARCARAH (UnB-Expansion)
*** Conteudo: modulo socketmsg.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 01/05/2009: criacao
*****************************************************************************/
/*! \file socketmsg.h 
* \brief Arquivo cabecalho do modulo exemplo. */
#ifndef SOCKETMSG_H
#define SOCKETMSG_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
#define SOCKETMSG_MAX_MESSAGE_DATA_FIELDS	10

#define SOCKETMSG_SERVERADDRESSMODE_NAME	1
#define SOCKETMSG_SERVERADDRESSMODE_IP		2

#define SOCKETMSG_MODE_SERVER			3
#define SOCKETMSG_MODE_CLIENT			4

typedef struct{
	pthread_mutex_t mutex;
	int 			mode;
	int 			serversocketfd;
	int 			clientsocketfd;
	struct 			sockaddr_in	serveraddress;
	struct 			sockaddr_in	clientaddress;
	unsigned char   *buffer;
	int 			bufferlen;
	int				flagverbose;
} socketmsg_t;

typedef struct{
	unsigned char   Header;
	unsigned char   *pData[SOCKETMSG_MAX_MESSAGE_DATA_FIELDS];
	int 			DataSizes[SOCKETMSG_MAX_MESSAGE_DATA_FIELDS];
	int 			NData;
} messsage_t;

// Prototipos de uso externo:
int socketmsg_server_init(int PortNumber, socketmsg_t *pSocketMsgStruct, int BufferLength, int flagverbose);
int socketmsg_client_init(char *pServer, int ServerAddressMode, int PortNumber, socketmsg_t *pSocketMsgStruct, int BufferLength, int flagverbose);
int socketmsg_close(socketmsg_t *pSocketMsgStruct);
int socketmsg_server_acceptconnection(socketmsg_t *pSocketMsgStruct);
int socketmsg_wait_message(socketmsg_t *pSocketMsgStruct, messsage_t *pMessage);
int socketmsg_send_message(socketmsg_t *pSocketMsgStruct, messsage_t *pMessage);

#ifdef __cplusplus
}
#endif 

#endif // SOCKETMSG_H
