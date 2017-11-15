/*******************************************************************************
* \file serialcom-sb16c1052pci.c
* \brief  Implementa modulo basico de acesso ao controlador de porta serial sb16c1052pci.
// Observacoes:
	- Deve instalar pciutils e libpci-dev
	- Consultado  /usr/share/doc/pciutils/examples/example.c
	- Usado man pcilib
	- http://alenitchev.wordpress.com/page/2/
	- lspci -vv verificar linha: 03:01.0 Serial controller: Systembase Co Ltd Device 4d02 (rev b0) (prog-if 02)
	- DB9 pinout:
	* RS 485 A Tx signal (+): pin 3 
	* RS 485 B Tx signal (-): pin 8 
	* RS 485 A Rx signal (+): pin 2 
	* RS 485 B Rx signal (-): pin 7 
*******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <sys/io.h> 
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/virtio_pci.h>
#include <pci/pci.h>
#include <pci/config.h>
#include <pci/header.h>
#include <pci/types.h>

#include "serialcom.h" 

#define PCI_VENDOR_ID_MULTIPORT    	0x14A1
#define PCI_DEVICE_ID_MP1       	0x4d01
#define PCI_DEVICE_ID_MP2       	0x4d02

/*! \var SEM *pComPortSemaphores[4] 
* Vetor de ponteiros para semaforos. Cada elemento desse vetor eh um ponteiro para o semaforo associado aa porta X, com X = 1, 2, 3 ou 4. Os semaforos de cada porta sao iniciados na chamada aa funcao serialcom_init(). Para uso interno pelas funcoes serialcom_semwait() e serialcom_semsignal(). */
RT_SEM pComPortSemaphores[4]; 

// funcões de uso interno apenas
int serialcom_pci_find_device(unsigned int vendor_id, unsigned int device_id, unsigned long *pbase_address, struct pci_access **ppacc);
void serialcom_pci_close_device(struct pci_access *pacc);

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

	unsigned long board_base_address;
	struct pci_access *pacc;

	if((ComPortNumber<0) || (ComPortNumber>2)){
		return SERIALCOM_ERROR_INCORRECTPORTNUMBER;
	}
	
	printf("\n*** probing PCI devices:");
	if(!serialcom_pci_find_device(PCI_VENDOR_ID_MULTIPORT, PCI_DEVICE_ID_MP2, &board_base_address, &pacc)){
		printf("\n    device not found !");
		return SERIALCOM_ERROR_DEVICENOTFOUND;
	}
	unsigned long board_port1_base_address;
	unsigned long board_port2_base_address;
	board_port1_base_address = board_base_address;
	board_port2_base_address = board_base_address+8;
	printf("\n*** board_port1_base_address: %Xh",(unsigned int)board_port1_base_address);
	printf("\n*** board_port2_base_address: %Xh",(unsigned int)board_port2_base_address);
	
	serialcom_pci_close_device(pacc);

	pSerialPortConfig->ComPortNumber = ComPortNumber; 
	pSerialPortConfig->ComPortAddress = board_base_address+8*(ComPortNumber-1); 
	

	pSerialPortConfig->ComPortBPS = ComPortBPS;
	pSerialPortConfig->FramePeriodUS = (1e7)/(((float)(ComPortBPS)));

	BPSClockDivisor = 921600 / ComPortBPS;
	BPSPrecision = (((double)(ComPortBPS)) - 921600.0/BPSClockDivisor)/((double)(ComPortBPS));
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
	outb(0x03, pSerialPortConfig->ComPortAddress + 3);	// LCR: 8 Bits, No Parity, 1 Stop Bit 
	outb(0xC7, pSerialPortConfig->ComPortAddress + 2);	// FIFO Control: 64 bytes TX e RX FIFO, limpa TX e RX fifos
	outb(0x0B, pSerialPortConfig->ComPortAddress + 4);	// MCR: Ativa DTR, RTS = 0, e OUT2 	

	char tmp_lcr;
	
	tmp_lcr = inb(pSerialPortConfig->ComPortAddress + 3);		 // get LCR initial value
	outb(tmp_lcr | 0xBF, pSerialPortConfig->ComPortAddress + 3); // LCR set 0xBF

	outb(0xA4, pSerialPortConfig->ComPortAddress + 0); 			 // select page 3
	// set ATR: 
	// 		* habilita controle automatico do driver TX, deixando a linha desativada quando não transmite.
	// 		* habilita controle automatico do driver RX, deixando a linha desativada quando não transmite.
//	outb(0x03 | (1<<5) | (1<<4), pSerialPortConfig->ComPortAddress + 1);  
	outb(0x03 | (1<<5) | (1<<4) | (1<<6), pSerialPortConfig->ComPortAddress + 1);  
//	outb(0x03 | (1<<5) | (1<<4) | (1<<7) | (1<<6), pSerialPortConfig->ComPortAddress + 1);
  
	outb(0xA5, pSerialPortConfig->ComPortAddress + 0); 			 // select page 4
	// set AFR: 
	// 		* habilita FIFO de 256 bytes.
	outb(0x01, pSerialPortConfig->ComPortAddress + 1);  

	outb(tmp_lcr, pSerialPortConfig->ComPortAddress + 3); 		 // LCR set to initial value

	sprintf(semname,"CoSem%i",ComPortNumber);
	//printf("\n semname = %s",semname);
	rt_sem_create(&pComPortSemaphores[ComPortNumber-1],semname,1,S_FIFO);
	return SERIALCOM_SUCCESS;

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
		serialcom_delayus(0.2*pSerialPortConfig->FramePeriodUS); 
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
		serialcom_delayus(0.2*pSerialPortConfig->FramePeriodUS);
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
	int status;

	if(MaximaEsperaUS<0) MaximaEsperaUS = 0; // Espera minima de 0 us.

	ElapsedTime = 0.0;
	while(1)
	{
		status = serialcom_status(pSerialPortConfig);
		if( (status & SERIALCOM_STATUSMASK_RX_DATA_READY) ){
//			printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
//			printf("\n Porta %i, Retornando estado %i", pSerialPortConfig->ComPortNumber, SERIALCOM_SUCCESS);
			return(SERIALCOM_SUCCESS);
		}
		if( (status & SERIALCOM_STATUSMASK_ERROR_RX_FIFO) ){
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
			return(SERIALCOM_ERROR_RX_FIFO);
		}
		if( (status & SERIALCOM_STATUSMASK_BREAK_INTERRUPT) ){
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
			return(SERIALCOM_ERROR_BREAK_INTERRUPT);
		}
		if( (status & SERIALCOM_STATUSMASK_FRAMING_ERROR) ){
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
			return(SERIALCOM_ERROR_FRAMING_ERROR);
		}
		if( (status & SERIALCOM_STATUSMASK_PARITY_ERROR) ){
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
			return(SERIALCOM_ERROR_PARITY_ERROR);
		}
		if( (status & SERIALCOM_STATUSMASK_OVERRUN_ERROR) ){
			*pData = inb(pSerialPortConfig->ComPortAddress + 0);
			return(SERIALCOM_ERROR_OVERRUN_ERROR);
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


int serialcom_pci_find_device(unsigned int vendor_id, unsigned int device_id, unsigned long *pbase_address, struct pci_access **ppacc)
{
	struct pci_dev *dev;
	unsigned int c;
	char namebuf[1024], *name;

	/* Get the pci_access structure */
	*ppacc = pci_alloc();
	/* Set all options you want -- here we stick with the defaults */
	/* Initialize the PCI library */
	pci_init(*ppacc);		
	/* We want to get the list of devices */
	pci_scan_bus(*ppacc);		
	/* Iterate over all devices */
	for(dev=(*ppacc)->devices; dev; dev=dev->next){
		/* Fill in header info we need */
		pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS | PCI_FILL_IRQ);	
		/* Read config register directly */
		c = pci_read_byte(dev, PCI_INTERRUPT_PIN);				
		printf("\n    %04x:%02x:%02x.%d vendor=%04x device=%04x class=%04x irq=%d (pin %d) base0=%lx",
			dev->domain, dev->bus, dev->dev, dev->func, dev->vendor_id, dev->device_id,
			dev->device_class, dev->irq, c, (long) dev->base_addr[0]);
		/* Look up and print the full name of the device */
		name = pci_lookup_name(*ppacc, namebuf, sizeof(namebuf), PCI_LOOKUP_DEVICE, dev->vendor_id, dev->device_id);
		printf(" (%s)", name);
		/* Connect */
		if((dev->vendor_id==vendor_id)&&(dev->device_id==device_id)){
			// recover base address
			*pbase_address = pci_read_long(dev, PCI_BASE_ADDRESS_0);
			*pbase_address &= PCI_BASE_ADDRESS_IO_MASK;

			return 1;
		}
	}
	/* Close everything */
	pci_cleanup(*ppacc);		
	
	return 0;
}

void serialcom_pci_close_device(struct pci_access *pacc)
{
	/* Close everything */
	pci_cleanup(pacc);		
}

