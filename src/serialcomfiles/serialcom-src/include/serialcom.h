/*******************************************************************************
* \file serialcom.h
* \brief  Implementa modulo basico de acesso � porta serial.
// Observa�oes:
//    - 
*******************************************************************************/
#ifndef SERIALCOM_H
#define SERIALCOM_H

#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
// Tipo correspondente  configurao da porta, que deve ficar no thread de chamada das funes:
/*! \typedef SERIALPORTCONFIG  
* Estrutura que mantm informaes relativas  configurao da porta serial. Cada thread com acesso a uma porta serial deve ter uma 
* varivel desse tipo, uma para cada porta.
*/
typedef struct{
	/*! Nome do dispositivo da porta serial, tal como /dev/tty0, /dev/ttyUSB0, etc. */
	char pComPortDevice[200]; 
	/*! Numero da porta serial, de 1 a 4. � um numero identificador sem rela��o com o numero do device */
	unsigned int ComPortNumber; 
	/*! Taxa de comunicao em BPS, de 2 a 115200 */
	unsigned int ComPortBPS;    
	/*! Descritor do dispositivo de comunicacao serial e estruturas associadas */
	int fd;
	struct termios oldtio,newtio;
	/*! Perodo em microsegundos correspondente a um frame */
	float FramePeriodUS; 
} SERIALPORTCONFIG, *PSERIALPORTCONFIG;

/*! \def SERIALCOM_USE_RS485 
* Se SERIALCOM_USE_RS485 esetiver em 1, a linha RTS ser colocada em nivel lgico 1 durante a transmissao 
* de cada frame de um byte. Em geral, isso  usado quando se desejar utilizar um driver RS485 com sinal 
* de controle pelo pino RTS. 
*/
#define SERIALCOM_USE_RS485 	0  

/*! \def SERIALCOM_MAXBPSPRECISION 
* Erro relativo maximo aceitavel para definir taxa de transmisso. Padr�o: 0.02, que corresponde a 2% de erro. 
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
#define SERIALCOM_ERROR_INVALIDBAUDRATE				6
#define SERIALCOM_ERROR_INVALIDDEVICE				7

// Mascaras de teste de status retornado por serialcom_status
#define SERIALCOM_STATUSMASK_ERROR_RX_FIFO 		0x80
#define SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS	0x40
#define SERIALCOM_STATUSMASK_EMPTY_TX_REGISTER	0x20
#define SERIALCOM_STATUSMASK_BREAK_INTERRUPT	0x10
#define SERIALCOM_STATUSMASK_FRAMING_ERROR		0x08
#define SERIALCOM_STATUSMASK_PARITY_ERROR		0x04
#define SERIALCOM_STATUSMASK_OVERRUN_ERROR		0x02
#define SERIALCOM_STATUSMASK_RX_DATA_READY		0x01

// Prototipos de uso externo:
int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, char *pComPortDevice, unsigned long int ComPortBPS);
int serialcom_close(PSERIALPORTCONFIG pSerialPortConfig);
void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig);
void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig);
int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData);
int serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS);

#ifdef __cplusplus
}
#endif 

#endif // SERIALCOM_H


