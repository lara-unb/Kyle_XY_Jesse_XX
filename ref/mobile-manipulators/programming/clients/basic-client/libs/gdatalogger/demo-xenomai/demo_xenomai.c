#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <termios.h>
#include <fcntl.h>

#include <native/task.h>
#include <native/timer.h>

#include "../gqueue.h"
#include "../gmatlabdatafile.h"
#include "../gdatalogger.h"

// periodo de amostragem
#define TASK1_PERIOD_IN_NANO 1000000
#define TASK2_PERIOD_IN_NANO 1000000

static int quittask = 0;
GDATALOGGER gDataLogger;
double MatA[3][1];
double MatB[4][2];

int kbhit(void);
int getch(void);

//Funcao periodica
void periodicThread_handler1(void *arg)
{
	double t0, tprevious, tempo, tic, toc;
	double Tsleep, T, y;

	printf("\n**** Tarefa 1.");

	rt_task_set_periodic(NULL, TM_NOW, TASK1_PERIOD_IN_NANO);

	t0 = ((double)(rt_timer_read()))/1e9;
	tempo = 0.0;

	while(!quittask)
	{	
		rt_task_wait_period(NULL);

		// Calculo das variaveis
		tprevious = tempo;
		tempo = ((double)(rt_timer_read()))/1e9 - t0;
		T = tempo - tprevious;
		y = sin(2*3.1415926*tempo);
		MatA[0][0] = y;
		MatA[1][0] = 2.0*y;
		MatA[2][0] = -y;
	
		// usleep
		tic = ((double)(rt_timer_read()))/1e9;
		rt_task_sleep(10000000);
		toc = ((double)(rt_timer_read()))/1e9;
		Tsleep = toc-tic;

		// Inserir na fila
		gDataLogger_InsertVariable(&gDataLogger,"Tsleep1",&Tsleep);
		gDataLogger_InsertVariable(&gDataLogger,"t1",&tempo);
		gDataLogger_InsertVariable(&gDataLogger,"T1",&T);
		gDataLogger_InsertVariable(&gDataLogger,"y1",&y);
		gDataLogger_InsertVariable(&gDataLogger,"MatA",&MatA[0][0]);

	}

	printf("\nEncerrando a Tarefa 1.");
}

void periodicThread_handler2(void *arg)
{
	double t0, tprevious, tempo, tic, toc;
	double Tsleep, T, y;
	int i,j;

	printf("\n**** Tarefa 2.");

	rt_task_set_periodic(NULL, TM_NOW, TASK2_PERIOD_IN_NANO);

	t0 = ((double)(rt_timer_read()))/1e9;
	tempo = 0.0;

	while(!quittask)
	{	
		rt_task_wait_period(NULL);

		// Calculo das variaveis
		tprevious = tempo;
		tempo = ((double)(rt_timer_read()))/1e9 - t0;
		T = tempo - tprevious;
		y = sin(2*3.1415926*tempo);
	
		for (i=0;i<4;++i){
			for (j=0;j<2;++j){
				MatB[i][j] = i*j;
			}
		}
		//printf("\n tempo  = %f, y = %f",tempo,y);
	
		// usleep
		tic = ((double)(rt_timer_read()))/1e9;
		rt_task_sleep(10000);
		toc = ((double)(rt_timer_read()))/1e9;
		Tsleep = toc-tic;

		// Inserir na fila
		gDataLogger_InsertVariable(&gDataLogger,"Tsleep2",&Tsleep);
		gDataLogger_InsertVariable(&gDataLogger,"t2",&tempo);
		gDataLogger_InsertVariable(&gDataLogger,"T2",&T);
		gDataLogger_InsertVariable(&gDataLogger,"y2",&y);
		gDataLogger_InsertVariable(&gDataLogger,"MatB",&MatB[0][0]);
	}

	printf("\nEncerrando a Tarefa 2.");
}

void catch_signal(int sig)
{
}

int main(int argc, char *argv[])
{
	RT_TASK task1;
	RT_TASK task2;
	int status,n;
	double t0,tempo;
		
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	// Data logger:
	if(!gDataLogger_Init(&gDataLogger,"matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return 1;
	}

	gDataLogger_DeclareVariable(&gDataLogger,"t1","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"t2","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"T1","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"T2","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"y1","m",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"y2","m",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"Tsleep1","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"Tsleep2","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"MatA","s",3,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"MatB","m",4,2,1000);	

	// Tarefas
	status = rt_task_create(&task1, "task1", 0, 99, 0);
	if (status != 0) {
		perror("Criacao do thread 1 falhou.\n");
		return 1;
	}
	status = rt_task_start(&task1, &periodicThread_handler1, NULL);
	if (status != 0) {
		perror("Inicializacao do thread 1 falhou.\n");
		return 1;
	}
	rt_task_slice(&task1,100000);

	status = rt_task_create(&task2, "task2", 0, 99, 0);
	if (status != 0) {
		perror("Criacao do thread 2 falhou.\n");
		return 1;
	}
	status = rt_task_start(&task2, &periodicThread_handler2, NULL);
	if (status != 0) {
		perror("Inicializacao do thread 2 falhou.\n");
		return 1;
	}
	rt_task_slice(&task2,100000);
	
	t0 = ((double)(rt_timer_read()))/1000000.0;
	n = 0;
	while(!kbhit()){
		usleep(20000);	
		gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
		if(++n>10){
			gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
			tempo = ((double)(rt_timer_read()))/1000000.0 - t0;
			printf("\n [%i] Atualizado arquivo Matlab em t = %f s",n,tempo);
			n = 0;
		}
	}

	quittask = 1;

	perror("\nEncerrando thread 1...");
	rt_task_join(&task1); 
	perror(" ok\n");
	perror("\nEncerrando thread 2...");
	rt_task_join(&task2); 
	perror(" ok\n");
	
	rt_task_delete(&task1);
	rt_task_delete(&task2);

	// Encerramento do data logger:
	gDataLogger_Close(&gDataLogger);
	
	return 0;
}

/**********************************************************************
 **** Gerenciamento do teclado
 *********************************************************************/
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}

int getch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}
