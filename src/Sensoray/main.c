/*-----------------------------------------------------------------------------------------------------/
/										 Sensoray code                                                 /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição: Código para leitura de enconder usando a biblioteca sensoray526 criada por G. A. Borges  /
/  e F. B. Cavalcanti                                                                                  /
/-----------------------------------------------------------------------------------------------------*/

// Bibliotecas
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
#include <iostream>
#include <sys/select.h>
#include "sensoray526.h"


// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init) 	if(cmd_init==0){printf("    Erro em %s",#cmd_init);return(0);}
#define MAIN_MODULE_CLOSE(cmd_close) 	if(cmd_close==0){printf("    Erro em %s",#cmd_close);}

// Protótipos das funções
int mode1_handler(void); //
int mode2_handler(void);
int mode3_handler(void);
int mode4_handler(void);

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

int kbhit(void)
{
  struct timeval tv;
  fd_set read_fd;

  /* Do not wait at all, not even a microsecond */
  tv.tv_sec=0;
  tv.tv_usec=0;

  /* Must be done first to initialize read_fd */
  FD_ZERO(&read_fd);

  /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
  FD_SET(0,&read_fd);

  /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
  if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */

  /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */
  
if(FD_ISSET(0,&read_fd))
    /* Character pending on stdin */
    return 1;

  /* no characters were pending */
  return 0;
}


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
	printf("\n   (3): ler encoder");
	printf("\n   (4): testar ADC");
	printf("\n\n   Opção: ");
	mode = getchar() - '0';
	printf("%i",mode);

	printf("\n\n*** Executando modo %i",mode);
	
	while(!flag_quit && !kbhit()){
		switch(mode){
			case 1: if(!mode1_handler()) flag_quit = 1; break;
			case 2: if(!mode2_handler()) flag_quit = 1; break;
			case 3: if(!mode3_handler()) flag_quit = 1; break;
			case 4: if(!mode4_handler()) flag_quit = 1; break;
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
		
	usleep(10000);

	
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

	return status;
}


int mode3_handler(void)
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

int mode4_handler(void)
{
	sensoray526_configure_AD(S526_ADC_ENABLE_CHANNEL_0|S526_ADC_ENABLE_CHANNEL_1|S526_ADC_ENABLE_CHANNEL_2|S526_ADC_ENABLE_CHANNEL_3|S526_ADC_ENABLE_CHANNEL_4|S526_ADC_ENABLE_CHANNEL_5|S526_ADC_ENABLE_CHANNEL_6|S526_ADC_ENABLE_CHANNEL_7);
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





/*-------------------------------Informações: ----------------------------------------
**** O contador reseta a contagem em 10s
****-------------------------------------------------------------------------------*/