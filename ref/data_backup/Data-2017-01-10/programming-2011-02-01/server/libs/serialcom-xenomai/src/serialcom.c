//***********************************************************************/
/* BIBLIOTECA DE FUNCOES PARA COMUNICACAO SERIAL UTILIZANDO RTAI	*/
/*									*/
/* LABORATORIO DE ROBOTICA E AUTOMACAO	- LARA				*/
/* DEPARTAMENTO DE ENGENHARIA ELETRICA	- ENE				*/
/* FACULDADE DE TECNOLOGIA		- FT				*/
/* UNIVERSIDADE DE BRASILIA		- UnB				*/
/*									*/
/************************************************************************/

/*! \file serialcomxenomai.c 
* \brief Arquivo com as funcoes da biblioteca serialcomxenomai. */
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <sys/io.h> 
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

#include "serialcom.h" 

/*! \def SERIALCOM_COMPORTADDRESS_1 
* Endereco base da porta serial COM1. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_1 0x3F8
/*! \def SERIALCOM_COMPORTADDRESS_2 
* Endereco base da porta serial COM2. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_2 0x2F8
/*! \def SERIALCOM_COMPORTADDRESS_3 
* Endereco base da porta serial COM3. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_3 0x3E8
/*! \def SERIALCOM_COMPORTADDRESS_4 
* Endereco base da porta serial COM4. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_4 0x2E8

/*! \var SEM *pComPortSemaphores[4] 
* Vetor de ponteiros para semaforos. Cada elemento desse vetor eh um ponteiro para o semaforo associado aa porta X, com X = 1, 2, 3 ou 4. Os semaforos de cada porta sao iniciados na chamada aa funcao serialcom_init(). Para uso interno pelas funcoes serialcom_semwait() e serialcom_semsignal(). */
RT_SEM pComPortSemaphores[4]; 

// macros
#define serialcom_delayus(a) rt_task_sleep((RTIME)((a)*1000))

/************* Rotinas Genericas de Manipulacao da porta serial com acesso externo *****************/
/*! \fn int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, unsigned long int ComPortBPS)
* Funcao que inicia a porta serial ComPortNumber com a taxa dada em BPS por ComPortBPS. Essa funcao dever ser chamada por cada thread 
* que tenha acesso  porta serial ComPortNumber. Seus argumentos de chamada so:
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORCONFIG.
* \param ComPortNumber Numero da porta serial, no intervalo de 1 a 4.
* \param ComPortBPS Taxa de comunicao em BPS, no intervalo de 2 a 115200.
* \return SERIALCOM_SUCCESS : Porta iniciada com sucesso. 
* \return SERIALCOM_ERROR_INCORRECTPORTNUMBER : Erro, corresponde a um ComPortNumber invlido.
* \return SERIALCOM_ERROR_MAXBPSPRECISION : Erro, a taxa ComPortBPS no pode ser realizada com erro inferior a. SERIALCOM_MAXBPSPRECISION. 
* \return SERIALCOM_ERROR_IOPL : Erro, corresponde a uma tentativa de executar o programa sem que se tenha acesso privilegiado de administrador a portas de E/S. 
*/
int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, unsigned long int ComPortBPS)
{
	unsigned int BPSClockDivisor;
	float BPSPrecision;
	char semname[50];

	pSerialPortConfig->ComPortNumber = ComPortNumber; 
	switch(ComPortNumber){
		case 0:
			pSerialPortConfig->ComPortAddress = 0x0400; 
		break;
		case 1:
			pSerialPortConfig->ComPortAddress = SERIALCOM_COMPORTADDRESS_1; 
		break;
		case 2:
			pSerialPortConfig->ComPortAddress = SERIALCOM_COMPORTADDRESS_2; 
		break;
		case 3:
			pSerialPortConfig->ComPortAddress = SERIALCOM_COMPORTADDRESS_3; 
		break;
		case 4:
			pSerialPortConfig->ComPortAddress = SERIALCOM_COMPORTADDRESS_4; 
		break;
		default:
			return SERIALCOM_ERROR_INCORRECTPORTNUMBER;
	}
	
	pSerialPortConfig->ComPortNumber = ComPortNumber; 
	pSerialPortConfig->ComPortBPS = ComPortBPS;
	pSerialPortConfig->FramePeriodUS = (1e7)/(((float)(ComPortBPS)));

	BPSClockDivisor = 115200 / ComPortBPS;
 	BPSPrecision = (((double)(ComPortBPS)) - 115200.0/BPSClockDivisor)/((double)(ComPortBPS));
	if(fabs(BPSPrecision) > SERIALCOM_MAXBPSPRECISION){
		return SERIALCOM_ERROR_MAXBPSPRECISION;
	}

        if(iopl(3)<0){
		printf("seriallib_iniciar: iopl error");
		return SERIALCOM_ERROR_IOPL;
	}

	outb(0, pSerialPortConfig->ComPortAddress + 1);		// Desativar interrupes 
	outb(0x80, pSerialPortConfig->ComPortAddress + 3);	// DLAB ON 
	outb((unsigned char)(BPSClockDivisor & 0xFF), pSerialPortConfig->ComPortAddress + 0);	// Baud Rate - DL Byte 
	outb((unsigned char)((BPSClockDivisor >> 8) & 0xFF), pSerialPortConfig->ComPortAddress + 1);	// Baud Rate - DH Byte 
	outb(0x03, pSerialPortConfig->ComPortAddress + 3);	// 8 Bits, No Parity, 1 Stop Bit 
	//outb(0x08, pSerialPortConfig->ComPortAddress + 3);	// 8 Bits, No Parity, 2 Stop Bits 
	outb(0xC7, pSerialPortConfig->ComPortAddress + 2);	// FIFO: 14 bytes, limpa TX e RX fifos, habilita
	outb(0x0B, pSerialPortConfig->ComPortAddress + 4);	// Ativa DTR, RTS = 0, e OUT2 	

	sprintf(semname,"CoSem%i",ComPortNumber);
	rt_sem_create(&pComPortSemaphores[ComPortNumber-1],semname,1,S_FIFO);

	return SERIALCOM_SUCCESS;
}

int serialcom_close(PSERIALPORTCONFIG pSerialPortConfig)
{
	rt_sem_delete(&pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);

	return SERIALCOM_SUCCESS;
}

/*! \fn void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que aguarda semforo para acessar a porta descrita por pSerialPortConfig. Juntamente com serialcom_semsignal, pode-se 
* garantir o acesso exclusivo de um thread  porta serial. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG..
* \warning Se uma determinada porta somente  gerenciada por um s thread, no h necessidade de se usar essas funes de semforo. As 
* funes de semforo tm somente utilizade em situaes em que mais de um thread pode acessar a porta serial X, com X = 1, 2, 3 ou 4.
* \warning Aps concluir o acesso  porta serial cedido por essa funcao, deve-se chamar serialcom_semsignal para liberar o semforo
*/
void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
{
	rt_sem_p(&pComPortSemaphores[pSerialPortConfig->ComPortNumber-1],TM_INFINITE);
}

/*! \fn void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que libera semforo que foi previamente cedido por serialcom_semwait para acessar a porta descrita por pSerialPortConfig.  
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG..
* \warning Se uma determinada porta somente  gerenciada por um s thread, no h necessidade de se usar essas funes de semforo. As 
* funes de semforo tm somente utilizade em situaes em que mais de um thread pode acessar a porta serial X, com X = 1, 2, 3 ou 4.
* \warning Aps concluir o acesso  porta serial cedido por essa funcao, deve-se chamar serialcom_semsignal para liberar o semforo
*/
void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
{
	rt_sem_v(&pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);
}

/*! \fn int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
* Funcao que envia um byte apontado por pData pela porta serial descrita por pSerialPortConfig.   
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG. Se SERIALCOM_USE_RS485 = 1, ento o sinal RTS ser colocado em nvel lgico 1 enquanto durar o frame do 
* byte enviado, permitindo assim ativar o driver externo de uma porta com conversor RS-485. Nessa situao, essa funcao somente retorna 
* quando o byte tiver sido enviado. Caso contrrio, a funcao somente escrever no buffer de sada o byte apontado por pData, retornando em * seguida.
* \param pData Ponteiro para o byte que ser enviado.
* \return SERIALCOM_SUCCESS : Dado escrito no registro de sada com sucesso. Entretanto, isso significa apenas que uma transmisso est em curso. Para se certificar de que o dado foi efetivamente transmitido, deve-se fazer uso da funcao serialcom_status()
* \return SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION : Situao de erro em que a funcao ficou aguardando por um perodo de at 5 frames para disponibilizao do registro de sada da porta
* \warning Essa funcao fica bloqueada enquanto o ltimo byte escrito no buffer de sada ainda no tiver sido enviado.
*/
int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
{
	int contador;
	#if SERIALCOM_USE_RS485
	unsigned char dado;
	#endif

	contador = 0;
	while(!(serialcom_status(pSerialPortConfig) & SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS))
	{
		serialcom_delayus(0.5*pSerialPortConfig->FramePeriodUS); 
		if(++contador>10) return(SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION);
	} // Espera fim da ultima transmissao

	#if SERIALCOM_USE_RS485
	dado = inb(pSerialPortConfig->ComPortAddress + 4);
	dado = dado & (~0x02);
	outb(dado, pSerialPortConfig->ComPortAddress + 4);  //setar RTS = 1 mantendo OUT2
	#endif
	
	outb(*pData, pSerialPortConfig->ComPortAddress + 0); // Envia.
	
	#if SERIALCOM_USE_RS485
//	serialcom_delayus(pSerialPortConfig->FramePeriodUS); // Espera o periodo de uma transmisso
	contador = 0;
	while(!(serialcom_status(pSerialPortConfig) & SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS))
	{
		serialcom_delayus(0.5*pSerialPortConfig->FramePeriodUS);
		if(++contador>10) return(SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION);
	} // Espera fim da ultima transmissao

	dado = inb(pSerialPortConfig->ComPortAddress + 4);
	dado = dado | (0x02);
	outb(dado, pSerialPortConfig->ComPortAddress + 4);  //setar RTS = 0 mantendo OUT2
	#endif

	return SERIALCOM_SUCCESS;
}

/*! \fn serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
* Funcao que aguarda um byte chegar pela porta serial descrita por pSerialPortConfig por um tempo mximo dado por MaximaEsperaUS, dado em 
* microsegundos. Se um dado chegar dentro do perodo dado por MaximaEsperaUS, o mesmo ser colocado na varivel apontada por pData. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada.
* \param pData Ponteiro para o byte recebido.
* \param MaximaEsperaUS Tempo mximo de espera pela chegada de um byte pela porta. Se MaximaEsperaUS 
* \return SERIALCOM_SUCCESS : Operao realizada com sucesso. Um byte foi recebido pela porta serial e se encontra disponvel na varivel 
* apontada por pData.
* \return SERIALCOM_ERROR_MAXWAITFORRECEPTION : Nenhum bayte chegou dentro do tempo estipulado por MaximaEsperaUS
* \warning Essa funcao fica bloqueada por at MaximaEsperaUS enquanto um byte no chegar.
*/
int serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
{
	double	ElapsedTime;

	if(MaximaEsperaUS<0) MaximaEsperaUS = 0; // Espera minima de 0 us.

	ElapsedTime = 0.0;
	while(1)
	{
		if( (serialcom_status(pSerialPortConfig) & SERIALCOM_STATUSMASK_RX_DATA_READY) ){
//			printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
//			printf("\n Porta %i, Retornando estado %i", pSerialPortConfig->ComPortNumber, SERIALCOM_SUCCESS);
			return(SERIALCOM_SUCCESS);
		}
		serialcom_delayus(0.2*pSerialPortConfig->FramePeriodUS);
		ElapsedTime += 0.2*pSerialPortConfig->FramePeriodUS;
//		printf("\n ElapsedTime = %f",ElapsedTime);
		if(ElapsedTime >= MaximaEsperaUS){
//			printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
//			printf("\n Porta %i, Retornando estado %i", pSerialPortConfig->ComPortNumber, SERIALCOM_ERROR_MAXWAITFORRECEPTION);
			return(SERIALCOM_ERROR_MAXWAITFORRECEPTION);  // Dado nao chegou no tempo estipulado.
		}
	}
}

/*! \fn int serialcom_status(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que l o registro de status da porta serial descrita por pSerialPortConfig. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada.
* \return O valor de retorno tem os bits setados conforme que o dado foi efetivamente enviadoos eventos que ocorreram com a porta serial, que podem ser testados usando um 
* teste lgico E bit a bit com as seguintes mscaras: 
* \return SERIALCOM_STATUSMASK_ERROR_RX_FIFO  
* \return SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS 
* \return SERIALCOM_STATUSMASK_EMPTY_TX_REGISTER
* \return SERIALCOM_STATUSMASK_BREAK_INTERRUPT
* \return SERIALCOM_STATUSMASK_FRAMING_ERROR
* \return SERIALCOM_STATUSMASK_PARITY_ERROR
* \return SERIALCOM_STATUSMASK_OVERRUN_ERROR
* \return SERIALCOM_STATUSMASK_RX_DATA_READY
* \return As mscaras acima correspondem a eventos que so detalhados em http://www.beyondlogic.org/serial/serial.htm 
*/
int serialcom_status(PSERIALPORTCONFIG pSerialPortConfig)
{
	return inb(pSerialPortConfig->ComPortAddress + 5);
}

/************* Rotinas Genericas de Manipulacao da porta serial com acesso interno *****************/

/*! \mainpage 
A biblioteca serialcomxenomai foi concebida para dar funcionalidade de comunicao serial para processos LINUX com a extenso de tempo real RTAI. Ela  disponibilizada na forma de cdigo fonte nos arquivos serialcomxenomai.c e serialcomxenomai.h. Essa biblioteca foi concebida para ser compatvel com processos com multiplos threads, e permite ainda que vrios threads acessem a mesma porta serial. No caso, esto implementadas funes para COM1, COM2, COM3 e COM4. E ainda, a biblioteca implementa funes de controle de acesso por semforo, o que permite que uma mesma porta serial possa ser acessada por um s thread por vez. Dependendendo do tipo de protocolo, o uso de semforos se faz necessrio. 

O projeto acompanha um exemplo no diretrio test. Para compilar o exemplo, basta fazer make. O resultado  o arquivo eval_serialcomxenomai. Antes de executar esse arquivo  necessrio pelo menos uma vez aps ter iniciado o sistema carregar os mdulos do RTAI. Para isso, basta executar o script loadmods.

*/


