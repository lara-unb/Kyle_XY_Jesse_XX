/*****************************************************************************
*** Arquivo: robotcommondefs.h
*** Conteudo: definições em comum entre cliente e servidor.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 12-01-2011: criacao
*****************************************************************************/
/*! \file robotcommondefs.h 
* \brief Arquivo cabecalho com definições em comum entre cliente e servidor. */
#ifndef ROBOTCOMMONDEFS_H
#define ROBOTCOMMONDEFS_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Portas de acesso no servidor:
#define ROBOTSERVER_PORTNUMBER_DATA			1024
#define ROBOTSERVER_PORTNUMBER_CAMERA		1025

// Tamanho dos buffers de mensagens entre servidor e cliente:
#define ROBOTCLIENT_MAXMSGSIZE				(2 * 1280 * 960 * 3 + 100)
#define ROBOTSERVER_MAXMSGSIZE				(ROBOTCLIENT_MAXMSGSIZE)

/*********** Protocolo de comunicação por socket *************/
// Definicoes de uso comum: mensagens do tipo SET
#define ROBOTPROTOCOL_MSGHEADER_SET_SERVOSMOVE			0x01

// Definicoes de uso comum: mensagens do tipo GET
#define ROBOTPROTOCOL_MSGHEADER_GET_TIME				0x81
#define ROBOTPROTOCOL_MSGHEADER_GET_IMAGE				0x82

// Definicoes de uso comum: mensagens de reconhecimento
#define ROBOTPROTOCOL_MSGHEADER_ERROR					0x00
#define ROBOTPROTOCOL_MSGHEADER_ACK						0xFF

// Definicoes de uso comum: tipos de variáveis usadas nas mensagens
#define ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS			8
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_0				0
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_1				1
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_2				2
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_3				3
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_4				4
#define ROBOTPROTOCOL_SERVO_MANIPULADOR_5				5
#define ROBOTPROTOCOL_SERVO_LEFTWHEELS					6
#define ROBOTPROTOCOL_SERVO_RIGHTWHEELS					7


typedef struct{
	int 			pulse_us[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
	char 			pulse_us_mask[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
	int 			pulse_speed_us_per_s[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
	char 			pulse_speed_us_per_s_mask[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
	int 			pulse_timeformove_ms[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
	char 			pulse_timeformove_ms_mask[ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS];
} servomovecommand_t;

static inline void SERVOMOVECOMMAND_RESETMASK(servomovecommand_t *pservomovecommand);
void SERVOMOVECOMMAND_RESETMASK(servomovecommand_t *pservomovecommand)
{
	int i;

	for(i=0;i<ROBOTPROTOCOL_SERVOCOMMANDS_MAXSERVOS;++i){
		pservomovecommand->pulse_us_mask[i] = 0;
		pservomovecommand->pulse_speed_us_per_s_mask[i] = 0;
		pservomovecommand->pulse_timeformove_ms_mask[i] = 0;
	}
}
	

#ifdef __cplusplus
}
#endif 

#endif // ROBOTCOMMONDEFS_H
