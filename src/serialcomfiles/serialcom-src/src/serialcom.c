/*******************************************************************************
* \file serialcom.c
* \brief  Implements the basic module to access the serial port.
// Observations:
//    - 
*******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <unistd.h> 	/* for libc5 */
#include <sys/io.h> 	/* for glibc */
#include <fcntl.h>		/* For O_* constants */
#include <sys/stat.h>	/* For mode constants */
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>

#include "serialcom.h" 

/*! \def SERIALCOM_COMPORTADDRESS_1 
* Serial port COM1's base address. Internal use. */
#define SERIALCOM_COMPORTADDRESS_1 0x3F8
/*! \def SERIALCOM_COMPORTADDRESS_2 
* Serial port COM2's base address. Internal use. */
#define SERIALCOM_COMPORTADDRESS_2 0x2F8
/*! \def SERIALCOM_COMPORTADDRESS_3 
* Serial port COM3's base address. Internal use. */
#define SERIALCOM_COMPORTADDRESS_3 0x3E8
/*! \def SERIALCOM_COMPORTADDRESS_4 
* Serial port COM4's base address. Internal use. */
#define SERIALCOM_COMPORTADDRESS_4 0x2E8

/*! \var SEM *pComPortSemaphores[4] 
* Semaphores' pointer vector.
* Each element in this vector is a semaphores' pointer associated to X port, with X = 1, 2, 3 or 4.
* The serialcom_init() function initializes each serial port's semaphores.
* Then, they are used internally by serialcom_semwait() and serialcom_semsignal(). */
sem_t *pComPortSemaphores[4] = {NULL, NULL, NULL, NULL}; 

// funcao de uso interno apenas
inline void serialcom_delayus(double timeus)
{
	struct timeval     timereset;
	struct timeval     time;
 
 	gettimeofday(&timereset, NULL);
	do  {
		usleep(timeus/4.0);
		gettimeofday(&time, NULL);
	} while (((time.tv_sec - timereset.tv_sec)*1e6 + (time.tv_usec - timereset.tv_usec)) < timeus);
}

/************* Serial port manipulation routines *****************/
/*! \fn int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, unsigned long int ComPortBPS)
* Initialize ComPortNumber with baud-rate given by ComPortBPS.
* This function must be called for each thread that has access to ComPortNumber.
* The arguments are:
* \param pSerialPortConfig Pointer to SERIALPORCONFIG that holds information about
* the serial port in context of the calling thread.
* This function handles access of several threads. For that, each one must have your own SERIALPORCONFIG struct.
* \param ComPortNumber Serial port number. It must be between [1, 4].
* \param ComPortBPS Communication rate in BPS. It must be between [2, 115200].
* \return SERIALCOM_SUCCESS : Successful initialization.
* \return SERIALCOM_ERROR_INCORRECTPORTNUMBER : Error, ComPortNumber invalid.
* \return SERIALCOM_ERROR_MAXBPSPRECISION : Error, ComPortBPS can not be used. ComPortBPS > SERIALCOM_MAXBPSPRECISION.
* \return SERIALCOM_ERROR_IOPL : Error, software tried to execute access without administrator rights for I/O ports.
*/
int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, char *pComPortDevice, unsigned long int ComPortBPS)
{
	#if SERIALCOM_USE_RS485
	int status;
	#endif
	char semname[50];
	int baudrate;

	strcpy(pSerialPortConfig->pComPortDevice, pComPortDevice);
	
	pSerialPortConfig->fd = open(pComPortDevice, O_RDWR ); 
	if (pSerialPortConfig->fd <0) {return(SERIALCOM_ERROR_INVALIDDEVICE); }

	tcgetattr(pSerialPortConfig->fd,&pSerialPortConfig->oldtio); /* save current port settings */

	switch(ComPortBPS){
		case 50:
			baudrate = B50;
			break;
		case 75:
			baudrate = B75;
			break;
		case 110:
			baudrate = B110;
			break;
		case 134:
			baudrate = B134;
			break;
		case 150:
			baudrate = B150;
			break;
		case 200:
			baudrate = B200;
			break;
		case 300:
			baudrate = B300;
			break;
		case 600:
			baudrate = B600;
			break;
		case 1200:
			baudrate = B1200;
			break;
		case 1800:
			baudrate = B1800;
			break;
		case 2400:
			baudrate = B2400;
			break;
		case 4800:
			baudrate = B4800;
			break;
		case 9600:
			baudrate = B9600;
			break;
		case 19200:
			baudrate = B19200;
			break;
		case 38400:
			baudrate = B38400;
			break;
		case 57600:
			baudrate = B57600;
			break;
		case 115200:
			baudrate = B115200;
			break;
		case 230400:
			baudrate = B230400;
			break;
		case 460800:
			baudrate = B460800;
			break;
		case 500000:
			baudrate = B500000;
			break;
		case 576000:
			baudrate = B576000;
			break;
		case 921600:
			baudrate = B921600;
			break;
		case 1000000:
			baudrate = B1000000;
			break;
		case 1152000:
			baudrate = B1152000;
			break;
		case 1500000:
			baudrate = B1500000;
			break;
		default:
			return SERIALCOM_ERROR_INVALIDBAUDRATE;
			break;
	}
	
	bzero((void*)&pSerialPortConfig->newtio, sizeof(pSerialPortConfig->newtio));
	pSerialPortConfig->newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
	pSerialPortConfig->newtio.c_iflag = IGNPAR;
	pSerialPortConfig->newtio.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	pSerialPortConfig->newtio.c_lflag = 0;
	 
	pSerialPortConfig->newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	pSerialPortConfig->newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

	tcflush(pSerialPortConfig->fd, TCIFLUSH);
	tcsetattr(pSerialPortConfig->fd,TCSANOW,&pSerialPortConfig->newtio);

	// printf("\n speed set to %X\n\n", cfgetispeed (&pSerialPortConfig->newtio)); exit(1);
	
	pSerialPortConfig->ComPortBPS = ComPortBPS;
	pSerialPortConfig->ComPortNumber = ComPortNumber;
	pSerialPortConfig->FramePeriodUS = (1e7)/(((float)(ComPortBPS)));

	fcntl(pSerialPortConfig->fd, F_SETFL, FNDELAY); // read functin returns imediatly (non-blocking mode)

	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status |= TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //setar RTS = 0.
	#endif

	sprintf(semname,"CoSem%i",ComPortNumber);
	pComPortSemaphores[ComPortNumber-1] = sem_open(semname,O_CREAT,S_IRUSR|S_IWUSR,1);
	// printf("\n Semaforo %s: %X\n",semname, pComPortSemaphores[ComPortNumber-1]);

	return SERIALCOM_SUCCESS;
}

int serialcom_close(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_close(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);

	return SERIALCOM_SUCCESS;
}

/*! \fn void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
* Function that waits for the semaphore to access the pSerialPortConfig.
* A thread can guarantee its sole access to a serial port using this function with serialcom_semsignal.
* \param pSerialPortConfig Pointer to SERIALPORCONFIG that holds information about
* the serial port in context of the calling thread.
* This function handles access of several threads. For that, each one must have your own SERIALPORCONFIG struct.
* \warning There is no need to use the semaphores' functions if a serial port is used by just one thread.
* The semaphores' functions must be used only when more than a thread uses the same serial port X.
* \warning After each access, the port must be released with serialcom_semsignal function.
*/
void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_wait(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);
}

/*! \fn void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
* Function releases the serial port semaphore, which was previously closed by serialcom_semwait function.
* \param pSerialPortConfig Pointer to SERIALPORCONFIG that holds information about
* the serial port in context of the calling thread.
* This function handles access of several threads. For that, each one must have your own SERIALPORCONFIG struct.
* \warning There is no need to use the semaphores' functions if a serial port is used by just one thread.
* The semaphores' functions must be used only when more than a thread uses the same serial port X.
* \warning After each access, the port must be released with serialcom_semsignal function.
*/
void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_post(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);
}

/*! \fn int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
* Function sends one byte pointed by pData through pSerialPortConfig.
* \param pSerialPortConfig Pointer to SERIALPORCONFIG that holds information about
* the serial port in context of the calling thread.
* This function handles access of several threads. For that, each one must have your own SERIALPORCONFIG struct.
* When SERIALCOM_USE_RS485 = 1, the function set RTS signal to logical level 1 while last the byte frame sent,
* allowing externally activation of the RS-485 driver.
* In this situation, the function will return only when the byte is sent.
* Otherwise, the function will write in output buffer the byte pointed by pData, returning after that.
* \param pData Pointer to the byte that has to be sent.
* \return SERIALCOM_SUCCESS : Data was written successfully in the output buffer.
* However, it means that the transmission is in progress. To certify the successful transmission, use serialcom_status.
* \return SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION : The function waited for the maximum time of 5 frames to
* the output buffer to be available.
* \warning This function pauses until the last byte written in the output buffer is has not been sent.
*/
int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
{

	#if SERIALCOM_USE_RS485
	int status;
	#endif

	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status &= ~TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //set RTS = 1.
	#endif
	
	if (write(pSerialPortConfig->fd, pData, 1) < 0) return SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION;
	
	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status |= TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //set RTS = 0.
	#endif

	return SERIALCOM_SUCCESS;
}

/*! \fn serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
* Function waits for a byte to be received in the serial port until MaximaEsperaUS, in microseconds.
* If a data arrive inside the time given by MaximaEsperaUS, pData will point to it.
* \param pSerialPortConfig Pointer to SERIALPORCONFIG that holds information about
* the serial port in context of the calling thread.
* \param pData Pointer to received byte.
* \param MaximaEsperaUS Maximum waiting time to arrive of a byte in the serial port.
* \return SERIALCOM_SUCCESS : Successful. A received byte is available and it is pointed by pData.
* \return SERIALCOM_ERROR_MAXWAITFORRECEPTION : Byte did not arrive inside the MaximaEsperaUS.
* \warning This function pauses until a byte arrive or MaximaEsperaUS.
*/
int serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
{
	double	ElapsedTime;
	int 	nbytesreceived;
	struct 	timeval timereset;
	struct 	timeval time;
 
	if(MaximaEsperaUS<0) MaximaEsperaUS = 0; // Espera minima de 0 us.

	ElapsedTime = 0.0;
	gettimeofday(&timereset, NULL);
	while(1)
	{
		nbytesreceived = read(pSerialPortConfig->fd,pData,1);
		if(nbytesreceived==1){
			// printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
			return(SERIALCOM_SUCCESS);
		}
		// serialcom_delayus(0.2*pSerialPortConfig->FramePeriodUS);
		gettimeofday(&time, NULL);
		ElapsedTime = ((time.tv_sec - timereset.tv_sec)*1e6 + (time.tv_usec - timereset.tv_usec));
		// printf("\n ElapsedTime = %f",ElapsedTime);
		if(ElapsedTime >= MaximaEsperaUS){
			// printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
			// printf("\n ElapsedTime = %f",ElapsedTime);
			// printf("\n Porta %i, Retornando estado %i", pSerialPortConfig->ComPortNumber, SERIALCOM_ERROR_MAXWAITFORRECEPTION);
			return(SERIALCOM_ERROR_MAXWAITFORRECEPTION);  // Dado nao chegou no tempo estipulado.
		}
	}
}

/*! \fn int serialcom_status(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que l o registro de status da porta serial descrita por pSerialPortConfig. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada.
* \return O valor de retorno tem os bits setados conforme que o dado foi efetivamente enviadoos eventos que ocorreram com a porta serial,
* que podem ser testados usando um 
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
	// undefined

/************* Rotinas Genericas de Manipulacao da porta serial com acesso interno *****************/

/*! \mainpage 
A biblioteca serialcom foi concebida para dar funcionalidade de comunicao serial para processos LINUX com a extenso de tempo real RTAI.
Ela  disponibilizada na forma de cdigo fonte nos arquivos serialcom.c e serialcom.h.
Essa biblioteca foi concebida para ser compatvel com processos com multiplos threads, e permite ainda que vrios threads acessem a mesma porta serial.
No caso, esto implementadas funes para COM1, COM2, COM3 e COM4.
E ainda, a biblioteca implementa funes de controle de acesso por semforo, o que permite que uma mesma porta serial possa ser acessada por um s thread por vez.
Dependendendo do tipo de protocolo, o uso de semforos se faz necessrio. 

O projeto acompanha um exemplo no diretrio test.
Para compilar o exemplo, basta fazer make.
O resultado  o arquivo eval_serialcom.
Antes de executar esse arquivo  necessrio pelo menos uma vez aps ter iniciado o sistema carregar os mdulos do RTAI.
Para isso, basta executar o script loadmods.

*/
