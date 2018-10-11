#define extern static // optimization to ams/io 
//#include <asm/io.h> //acess linux I/O
//#include <unistd.h> //ioperm - used to give permission to the acess the ports

//------------------------------------------------------
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/io.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>

#include <sys/mman.h>

#include "sensoray526.h"
//------------------------------------------------------

// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

// Cabecalhos especificos do modulo:
void catch_signal(int sig);
int mode1_handler(void);
int mode2_handler(void);
int mode3_handler(void);
int mode4_handler(void);
int mode5_handler(void);
int mode6_handler(void);

struct{
	struct timeval time;
	struct timeval timereset;
} tictocctrl;

void tic(void)
{
	gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
	gettimeofday(&tictocctrl.time, NULL);
	return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec)*1e-6);
}

// Variáveis do modulo:


//Portas
#define S526_IOSIZE 0x40
#define S526_NUM_PORTS 27


//Define registradores
/*#define REG_TCR 0x00
#define REG_WDC 0x02
#define REG_DAC 0x04
#define REG_ADC 0x06
#define REG_ADD 0x08
#define REG_DIO 0x0A
#define REG_IER 0x0C
#define REG_ISR 0x0E
#define REG_MSC 0x10
#define REG_C0L 0x12
#define REG_C0H 0x14
#define REG_C0M 0x16
#define REG_C0C 0x18
#define REG_C1L 0x1A
#define REG_C1H 0x1C
#define REG_C1M 0x1E
#define REG_C1C 0x20
#define REG_C2L 0x22
#define REG_C2H 0x24
#define REG_C2M 0x26
#define REG_C2C 0x28
#define REG_C3L 0x2A
#define REG_C3H 0x2C
#define REG_C3M 0x2E
#define REG_C3C 0x30
#define REG_EED 0x32
#define REG_EEC 0x34*/



// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

// Cabecalhos especificos do modulo:
void catch_signal(int sig);
int mode1_handler(void);
int mode2_handler(void);
int mode3_handler(void);
int mode4_handler(void);
int mode5_handler(void);
int mode6_handler(void);

struct{
	struct timeval time;
	struct timeval timereset;
} tictocctrl;

void tic(void)
{
	gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
	gettimeofday(&tictocctrl.time, NULL);
	return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec)*1e-6);
}

// Variáveis do modulo:

int main (int argc, char *argv[])
{       
	int flag_quit = 0;
	int mode = 0;
	
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	printf("\n\n\n\n***************************************************************");
	printf("\n**** Rotinas de teste do modulo sensoray526");
	printf("\n***************************************************************\n");
	printf("\n");

	// robot module:
	printf("\n*** Iniciando o modulo sensoray526...");
	MAIN_MODULE_INIT(sensoray526_init());

	
	printf("\n*** Escolha uma opção:");
	printf("\n   (1): testar DIO");
	printf("\n   (2): testar timer em modo de contagem regressiva");
	printf("\n   (4): configurar encoder");
	printf("\n   (5): ler encoder");
	printf("\n   (6): testar ADC");
	printf("\n\n   Opção: ");
	mode = getch() - '0';
	printf("%i",mode);

	printf("\n\n*** Executando modo %i",mode);
	
	while(!flag_quit && !kbhit()){
		switch(mode){
			case 1: if(!mode1_handler()) flag_quit = 1; break;
			case 2: if(!mode2_handler()) flag_quit = 1; break;
			case 3: if(!mode3_handler()) flag_quit = 1; break;
			case 4: if(!mode4_handler()) flag_quit = 1; break;
			case 5: if(!mode5_handler()) flag_quit = 1; break;
			case 6: if(!mode6_handler()) flag_quit = 1; break;
			default:
				printf("\n Modo não implementado"); flag_quit = 1;
		}
	}

	printf("\n*** Encerrando o modulo robot...");
	MAIN_MODULE_CLOSE(sensoray526_close());
	
	printf("\n\n");
	fflush(stdout); // mostra todos printfs pendentes.
    return 1;
}

void catch_signal(int sig)
{
}

int mode1_handler(void)
{
	float texec;
	int status;
	static int msgcounter = 0;
		
	// Sleep
	usleep(10000);

	// Envia comando para DIO:
	tic(); 
	status = sensoray526_set_dio(~sensoray526_get_dio(0xFF), 0xFF); 
	texec = toc(); 
	if(++msgcounter > 50){
		msgcounter = 0;
		if(status){
			printf("\n sensoray526_get_dio e sensoray526_set_dio executados com sucesso em %f ms",texec*1e3);
		} else{
			printf("\n sensoray526_get_dio e sensoray526_set_dio falharam");
		}
	}
	
	return status;
}

int mode2_handler(void)
{
	float texec;
	int status,i;
	static int msgcounter = 0;
	static float delay_s = 1e-03;
	static unsigned int moderegvalue = 0, regvalue;
		
	// Sleep
	usleep(10000);

	// Programa timer para gerar atraso de 1ms:

/*	
	// Step 1. Load the preload register PR0 with 0x13C68. First, set the Counter Mode register.
	// select PR0 as a target for Preload register access
	// set operating mode, don’t enable count yet
	moderegvalue = 	S526_REG_CxM_PRELOADREGISTER_PR0 |\
					S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE |\
					S526_REG_CxM_COUNTDIRECTION_DOWN |\
					S526_REG_CxM_CLOCKSOURCE_INTERNAL |\
					S526_REG_CxM_COUNTENABLE_HARDWARE |\
					S526_REG_CxM_HARDWARECOUNTENABLE_NOTRCAP |\
					S526_REG_CxM_AUTOPRELOAD_DISABLE |\  
					S526_REG_CxM_COUTPOLARITY_INVERTED |\
					S526_REG_CxM_COUTSOURCE_RCAP;
	sensoray526_write_register (regvalue, S526_REG_C3M); //load Counter Mode register
	regvalue = (unsigned int)(delay_s*27e6);
	sensoray526_write_register (regvalue >> 16, S526_REG_C3H); //load Preload Register high word
	sensoray526_write_register (regvalue & 0xFFFF, S526_REG_C3L); //load Preload Register low word

	// Step 2. Reset the counter (to clear RTGL), load the counter from Preload Register 0.
	sensoray526_write_register (0x8000, S526_REG_C3C); //reset the counter
	sensoray526_write_register (0x4000, S526_REG_C3C); //load the counter from PR0

	// Step 3. Reset RCAP (fires one-shot).
	sensoray526_write_register (0x08, S526_REG_C3C);

/*	// Step 1. Load the preload register PR0 with 0x13C68. First, set the Counter Mode register.
	// select PR0 as a target for Preload register access
	// set operating mode, don’t enable count yet
	moderegvalue = 	S526_REG_CxM_PRELOADREGISTER_PR0 |\
					S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE |\
					S526_REG_CxM_COUNTDIRECTION_DOWN |\
					S526_REG_CxM_CLOCKSOURCE_INTERNAL |\
					S526_REG_CxM_COUNTENABLE_DISABLED |\
					S526_REG_CxM_HARDWARECOUNTENABLE_CEN |\
					S526_REG_CxM_AUTOPRELOAD_DISABLE;
	sensoray526_write_register (moderegvalue, S526_REG_C3M); //load Counter Mode register
	regvalue = (unsigned int)(delay_s*27e6);
	sensoray526_write_register (regvalue >> 16, S526_REG_C3H); //load Preload Register high word
	sensoray526_write_register (regvalue & 0xFFFF, S526_REG_C3L); //load Preload Register low word

	// Step 2. Reset the counter (to clear RTGL), load the counter from Preload Register 0.
	sensoray526_write_register (0x4000, S526_REG_C3C); // load the counter from PR0
	sensoray526_write_register (0x0008, S526_REG_C3C); // reset flag

	// Step 3. Reset RCAP (fires one-shot).
	printf("\n moderegvalue = %X",moderegvalue);
	moderegvalue = (moderegvalue & (~(3 << 7))) | S526_REG_CxM_COUNTENABLE_ENABLED;
	sensoray526_write_register (moderegvalue, S526_REG_C3M);
	printf("\n moderegvalue = %X",moderegvalue);
	*/
	
	if((delay_s+=0.00001) > 9.0e-3) delay_s = 1e-3;

	moderegvalue = 	S526_REG_CxM_PRELOADREGISTER_PR0 |\
					S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE |\
					S526_REG_CxM_COUNTDIRECTION_DOWN |\
					S526_REG_CxM_CLOCKSOURCE_INTERNAL |\
					S526_REG_CxM_COUNTENABLE_DISABLED |\
					S526_REG_CxM_HARDWARECOUNTENABLE_CEN |\
					S526_REG_CxM_AUTOPRELOAD_RO |\  
					S526_REG_CxM_COUTPOLARITY_NORMAL |\
					S526_REG_CxM_COUTSOURCE_RTGL;
	printf("\n moderegvalue = %X",moderegvalue);
					
	regvalue = (unsigned int)(delay_s*27e6);
	sensoray526_write_register (moderegvalue, 0x16); //load Counter Mode register
	sensoray526_write_register (regvalue >> 16, 0x14); //load Preload Register 0 high word
	sensoray526_write_register (regvalue & 0xFFFF, 0x12); //load Preload Register 0 low word
	
	moderegvalue = 	S526_REG_CxM_PRELOADREGISTER_PR1 |\
					S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE |\
					S526_REG_CxM_COUNTDIRECTION_DOWN |\
					S526_REG_CxM_CLOCKSOURCE_INTERNAL |\
					S526_REG_CxM_COUNTENABLE_DISABLED |\
					S526_REG_CxM_HARDWARECOUNTENABLE_CEN |\
					S526_REG_CxM_AUTOPRELOAD_RO |\  
					S526_REG_CxM_COUTPOLARITY_NORMAL |\
					S526_REG_CxM_COUTSOURCE_RTGL;
	printf("\n moderegvalue = %X",moderegvalue);

	regvalue = (unsigned int)((10e-3 - delay_s)*27e6);
	sensoray526_write_register (moderegvalue, 0x16); //load Counter Mode register
	sensoray526_write_register (regvalue >> 16, 0x14); //load Preload Register 0 high word
	sensoray526_write_register (regvalue & 0xFFFF, 0x12); //load Preload Register 0 low word

	moderegvalue = 	S526_REG_CxM_PRELOADREGISTER_PR1 |\
					S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE |\
					S526_REG_CxM_COUNTDIRECTION_DOWN |\
					S526_REG_CxM_CLOCKSOURCE_INTERNAL |\
					S526_REG_CxM_COUNTENABLE_ENABLED |\
					S526_REG_CxM_HARDWARECOUNTENABLE_CEN |\
					S526_REG_CxM_AUTOPRELOAD_RO |\  
					S526_REG_CxM_COUTPOLARITY_NORMAL |\
					S526_REG_CxM_COUTSOURCE_RTGL;
	printf("\n moderegvalue = %X",moderegvalue);

	regvalue = (unsigned int)((10e-3 - delay_s)*27e6);
	sensoray526_write_register (moderegvalue, 0x16); //load Counter Mode register

	return 1;
	printf("\n***********************************");
	tic();
	sensoray526_set_dio(0xFF, 0xFF); 
	while(!(sensoray526_read_register(S526_REG_C3C) & 0x0008)){
		regvalue = sensoray526_read_register(S526_REG_C3L);
		regvalue |= (sensoray526_read_register(S526_REG_C3H) << 16);
		printf("\n %X : %i",sensoray526_read_register(S526_REG_C3C),regvalue);
		if(toc() > 5) return 0;
	}
	sensoray526_set_dio(0x00, 0xFF); 

//	return 0;
/*	tic(); 
	status = sensoray526_set_dio(~sensoray526_get_dio(0xFF), 0xFF); 
	texec = toc(); 
	if(++msgcounter > 50){
		msgcounter = 0;
		if(status){
			printf("\n sensoray526_get_dio e sensoray526_set_dio executados com sucesso em %f ms",texec*1e3);
		} else{
			printf("\n sensoray526_get_dio e sensoray526_set_dio falharam");
		}
	}*/
	
	return status;
}

int mode3_handler(void)
{
//	printf("\n Modo não implementado"); return 0;

sensoray526_write_register (0x1C85, 0x16); //load Counter Mode register
sensoray526_write_register (0x0003, 0x14); //load Preload Register 0 high word
sensoray526_write_register (0x4BC0, 0x12); //load Preload Register 0 low word
sensoray526_write_register (0x5C85, 0x16); //load Counter Mode register
sensoray526_write_register (0x0000, 0x14); //load Preload Register 0 high word
sensoray526_write_register (0xD2F0, 0x12); //load Preload Register 0 low word	
	// Sleep
	usleep(1000000);
	
	return 1;
}

int mode4_handler(void)
{

	
	// Sleep
	usleep(1000000);
	
	return 1;
}

int mode5_handler(void)
{
	unsigned char counter=0;
	//Configura encoder 0
	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);
	sensoray526_configure_encoder(2);
	sensoray526_configure_encoder(3);
	sensoray526_reset_counter(0);
	sensoray526_reset_counter(1);
	sensoray526_reset_counter(2);
	sensoray526_reset_counter(3);
	
	for(counter=0;counter<100;counter++)
	{
		//Le o encoder 0
		printf("\n Encoder 0: %X (%d)",sensoray526_read_counter(0),sensoray526_read_counter(0));
		printf("\n Encoder 1: %X (%d)",sensoray526_read_counter(1),sensoray526_read_counter(1));
		printf("\n Encoder 2: %X (%d)",sensoray526_read_counter(2),sensoray526_read_counter(2));
		printf("\n Encoder 3: %X (%d)",sensoray526_read_counter(3),sensoray526_read_counter(3));
	
		// Sleep
		usleep(100000);
	}
	
	return 1;
}

int mode6_handler(void)
{
	sensoray526_configure_AD(S526_ADC_ENABLE_CHANNEL_0|S526_ADC_ENABLE_CHANNEL_1|S526_ADC_ENABLE_CHANNEL_2|S526_ADC_ENABLE_CHANNEL_3|S526_ADC_ENABLE_CHANNEL_4|S526_ADC_ENABLE_CHANNEL_5|S526_ADC_ENABLE_CHANNEL_6|S526_ADC_ENABLE_CHANNEL_7);
	//sensoray526_configure_AD(S526_ADC_ENABLE_CHANNEL_6|S526_ADC_ENABLE_CHANNEL_7);
	//sensoray526_configure_AD(S526_ADC_ENABLE_CHANNEL_0);
	sensoray526_perform_AD_conversion();
	usleep(10000);
	printf("\n AD 0: %X (%d) = %f V",sensoray526_read_AD_raw(0),sensoray526_read_AD_raw(0),sensoray526_read_AD_voltage(0));
	printf("\n AD 1: %X (%d) = %f V",sensoray526_read_AD_raw(1),sensoray526_read_AD_raw(1),sensoray526_read_AD_voltage(1));
	printf("\n AD 2: %X (%d) = %f V",sensoray526_read_AD_raw(2),sensoray526_read_AD_raw(2),sensoray526_read_AD_voltage(2));
	printf("\n AD 3: %X (%d) = %f V",sensoray526_read_AD_raw(3),sensoray526_read_AD_raw(3),sensoray526_read_AD_voltage(3));
	printf("\n AD 4: %X (%d) = %f V",sensoray526_read_AD_raw(4),sensoray526_read_AD_raw(4),sensoray526_read_AD_voltage(4));
	printf("\n AD 5: %X (%d) = %f V",sensoray526_read_AD_raw(5),sensoray526_read_AD_raw(5),sensoray526_read_AD_voltage(5));
	printf("\n AD 6: %X (%d) = %f V",sensoray526_read_AD_raw(6),sensoray526_read_AD_raw(6),sensoray526_read_AD_voltage(6));
	printf("\n AD 7: %X (%d) = %f V",sensoray526_read_AD_raw(7),sensoray526_read_AD_raw(7),sensoray526_read_AD_voltage(7)*2.0504);
	printf("\n");
	usleep(500000);
	return 1;
}
