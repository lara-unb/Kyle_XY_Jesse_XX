//***********************************************************************/
/* BIBLIOTECA DE FUNCOES PARA COMUNICACAO SERIAL UTILIZANDO RTAI	*/
/*									*/
/* LABORATORIO DE ROBOTICA E AUTOMACAO	- LARA				*/
/* DEPARTAMENTO DE ENGENHARIA ELETRICA	- ENE				*/
/* FACULDADE DE TECNOLOGIA		- FT				*/
/* UNIVERSIDADE DE BRASILIA		- UnB				*/
/*									*/
/************************************************************************/

/*! \file serialcomxenomai.h 
* \brief Arquivo cabealho da biblioteca serialcomxenomai. */

#ifndef SERIALCOM_H
#define SERIALCOM_H

// Todas as funções podem ser referenciadas por serialcom_ em vez de serialcom_:
#define serialcom_ serialcom_

// Tipo correspondente  configurao da porta, que deve ficar no thread de chamada das funes:
/*! \typedef SERIALPORTCONFIG  
* Estrutura que mantm informaes relativas  configurao da porta serial. Cada thread com acesso a uma porta serial deve ter uma 
* varivel desse tipo, uma para cada porta.
*/
typedef struct{
	/*! Nmero da porta serial, de 1 a 4 */
	unsigned int ComPortNumber; 
	/*! Taxa de comunicao em BPS, de 2 a 115200 */
	unsigned int ComPortBPS;    
	/*! Endereo base da porta de comunicao serial */
	unsigned int ComPortAddress;
	/*! Perodo em microsegundos correspondente a um frame */
	float FramePeriodUS; 
} SERIALPORTCONFIG, *PSERIALPORTCONFIG;

/*! \def SERIALCOM_USE_RS485 
* Se SERIALCOM_USE_RS485 esetiver em 1, a linha RTS ser colocada em nvel lgico 1 durante a trnamisso de cada frame de um byte. Em geral, isso  usado quando se desejar utilizar um driver RS485 com sinal de controle pelo pino RTS. 
*/
#define SERIALCOM_USE_RS485 	1  

/*! \def SERIALCOM_MAXBPSPRECISION 
* Erro relativo maximo aceitavel para definir taxa de transmisso. PadrSe SERIALCOM_USE_RS485 esetiver em 1, a linha RTS ser colocada em nvel lgico 1 durante a trnamisso de cada frame de um byte. Em geral, isso  usado quando se desejar utilizar um driver RS485 com sinal de controle pelo pino RTSo: 0.02, que corresponde a 2% de erro. 
*/
//  
#define SERIALCOM_MAXBPSPRECISION	0.02	

// Retorno das funes
#define SERIALCOM_SUCCESS	 						0
#define SERIALCOM_ERROR_IOPL 						1
#define SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION	2
#define SERIALCOM_ERROR_MAXWAITFORRECEPTION			3
#define SERIALCOM_ERROR_MAXBPSPRECISION				4
#define SERIALCOM_ERROR_INCORRECTPORTNUMBER			5
#define SERIALCOM_ERROR_DEVICENOTFOUND				6
#define SERIALCOM_ERROR_RX_FIFO						7
#define SERIALCOM_ERROR_BREAK_INTERRUPT				8
#define SERIALCOM_ERROR_FRAMING_ERROR				9
#define SERIALCOM_ERROR_PARITY_ERROR				10
#define SERIALCOM_ERROR_OVERRUN_ERROR				11

// Mascaras de teste de status retornado por serialcom_status
#define SERIALCOM_STATUSMASK_ERROR_RX_FIFO 		0x80
#define SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS		0x40
#define SERIALCOM_STATUSMASK_EMPTY_TX_REGISTER		0x20
#define SERIALCOM_STATUSMASK_BREAK_INTERRUPT		0x10
#define SERIALCOM_STATUSMASK_FRAMING_ERROR		0x08
#define SERIALCOM_STATUSMASK_PARITY_ERROR		0x04
#define SERIALCOM_STATUSMASK_OVERRUN_ERROR		0x02
#define SERIALCOM_STATUSMASK_RX_DATA_READY		0x01

//Prototipos
int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, unsigned long int ComPortBPS);
int serialcom_close(PSERIALPORTCONFIG pSerialPortConfig);
int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData);
int serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS);
int serialcom_status(PSERIALPORTCONFIG pSerialPortConfig);
void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig);
void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig);

#endif
