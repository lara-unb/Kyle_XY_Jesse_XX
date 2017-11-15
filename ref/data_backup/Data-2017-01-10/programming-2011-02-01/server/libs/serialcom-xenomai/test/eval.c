#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include "../src/serialcom.h" 

// periodo de amostragem
#define TASK1_PERIOD_IN_NANO 30000000
#define TASK2_PERIOD_IN_NANO 30000000

static int quittask = 0;

//	unsigned int bps = 300;
//	unsigned int bps = 9600;
//	unsigned int bps = 57600;
	unsigned int bps = 115200;
//	unsigned int bps = 230400;
//	unsigned int bps = 460800;
//	unsigned int bps = 921600;

//Funcao periodica
void periodicThread_handler1(void *arg)
{
	int status;
	int i;
	RTIME tic, toc;
	double timetx,timerx;
	unsigned char datain, dataout = 0;
	SERIALPORTCONFIG SerialPortConfig; 

	printf("\n**** Tarefa 1.");
	printf("\nIniciando a porta serial ...");
	status = serialcom_init(&SerialPortConfig, 1, bps);
	if ( status ==  SERIALCOM_SUCCESS){
		printf(" ok!");
	} else {
		printf(" Erro (%i)",status);
		return;
	}

	// rt_task_set_mode(0,T_RRB ,NULL); versores do xenomai anteriores a 2.5.0
	rt_task_set_periodic(NULL, TM_NOW, TASK1_PERIOD_IN_NANO);
	while(!quittask)
	{	
		rt_task_wait_period(NULL);

		++dataout;
		tic = rt_timer_read();
		serialcom_semwait(&SerialPortConfig);
		for(i=0;i<1;++i){
			serialcom_sendbyte(&SerialPortConfig, &dataout);
		}
		serialcom_semsignal(&SerialPortConfig);
		toc = rt_timer_read();
		timetx = (double)(toc-tic);

		tic = rt_timer_read();
		serialcom_semwait(&SerialPortConfig);
		status = serialcom_receivebyte(&SerialPortConfig, &datain, 1000.0);
		serialcom_semsignal(&SerialPortConfig);
		toc = rt_timer_read();
		timerx = (double)(toc-tic);
		if ( status == SERIALCOM_SUCCESS){
			printf("\nTask1: Dado enviado = %2X (%.2f us) -  Dado recebido = %X (%.2f us)",dataout,timetx/1000.0,datain,timerx/1000.0);
		}
		else{
			printf("\nTask1: Dado enviado = %2X (%.2f us) -  Nada foi recebido em %.2f us (status = %i)",dataout,timetx/1000.0,timerx/1000.0,status);
		}
	}

	printf("\nEncerrando a Tarefa 1...");
	serialcom_close(&SerialPortConfig);
	printf(" ok!");
}

void periodicThread_handler2(void *arg)
{
	int status;
	int i;
	RTIME tic, toc;
	double timetx,timerx;
	unsigned char datain, dataout;
	SERIALPORTCONFIG SerialPortConfig; 

	printf("\n**** Tarefa 2.");
	printf("\nIniciando a porta serial ...");
	status = serialcom_init(&SerialPortConfig, 2, bps);
	if ( status ==  SERIALCOM_SUCCESS){
		printf(" ok!");
	} else {
		printf(" Erro (%i)",status);
		return;
	}

	// rt_task_set_mode(0,T_RRB ,NULL); versores do xenomai anteriores a 2.5.0
	rt_task_set_periodic(NULL, TM_NOW, TASK2_PERIOD_IN_NANO);

	while(!quittask)
	{	
		rt_task_wait_period(NULL);

		dataout = 0xFF;
		tic = rt_timer_read();
		serialcom_semwait(&SerialPortConfig);
		for(i=0;i<5;++i)
			serialcom_sendbyte(&SerialPortConfig, &dataout);
		serialcom_semsignal(&SerialPortConfig);
		toc = rt_timer_read();
		timetx = (double)(toc-tic);

		tic = rt_timer_read();
		serialcom_semwait(&SerialPortConfig);
		status = serialcom_receivebyte(&SerialPortConfig, &datain, 1000.0);
		serialcom_semsignal(&SerialPortConfig);
		toc = rt_timer_read();
		timerx = (double)(toc-tic);
		if ( status == SERIALCOM_SUCCESS){
			printf("\nTask2: Dado enviado = %2X (%.2f us) -  Dado recebido = %X (%.2f us)",dataout,timetx/1000.0,datain,timerx/1000.0);
		}
		else{
			printf("\nTask2: Dado enviado = %2X (%.2f us) -  Nada foi recebido em %.2f us (status = %i)",dataout,timetx/1000.0,timerx/1000.0,status);
		}
	}

	printf("\nEncerrando a Tarefa 2...");
	serialcom_close(&SerialPortConfig);
	printf(" ok!");
}

void catch_signal(int sig)
{
}

int main(int argc, char *argv[])
{
	RT_TASK task1;
	RT_TASK task2;
	int status;
		
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	status = rt_task_create(&task1, "task1", 0, 99, T_JOINABLE);
	if (status != 0) {
		perror("Criacao do thread 1 falhou.\n");
		exit(1);
	}
	status = rt_task_start(&task1, &periodicThread_handler1, NULL);
	if (status != 0) {
		perror("Inicializacao do thread 1 falhou.\n");
		exit(1);
	}
	rt_task_slice(&task1,100000);

/*	status = rt_task_create(&task2, "task2", 0, 99, T_JOINABLE);
	if (status != 0) {
		perror("Criacao do thread 2 falhou.\n");
		exit(1);
	}
	status = rt_task_start(&task2, &periodicThread_handler2, NULL);
	if (status != 0) {
		perror("Inicializacao do thread 2 falhou.\n");
		exit(1);
	}
	rt_task_slice(&task2,100000);
*/	
	getchar();

	quittask = 1;

	perror("\nEncerrando thread 1...");
	rt_task_join(&task1); 
	perror(" ok\n");
	perror("\nEncerrando thread 2...");
//	rt_task_join(&task2); 
	perror(" ok\n");
	
	rt_task_delete(&task1);
	rt_task_delete(&task2);
	
	return 0;
}
