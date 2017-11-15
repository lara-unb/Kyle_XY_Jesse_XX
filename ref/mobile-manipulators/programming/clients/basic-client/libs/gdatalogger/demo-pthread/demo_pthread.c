/* gcc -Wall -D_REENTRANT -D_POSIX_TIMERS timer.c -o timer -lrt */

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

#include "time.h"

#include "../gqueue.h"
#include "../gmatlabdatafile.h"
#include "../gdatalogger.h"

#define	SPEAKER_CNTRL	0x61

#define ENABLE_SPEAKER_TEST 0

#define TASK_PERIOD_US  10000

double MatA[3][1];
double MatB[4][2];

void timer_start (void);
void timer_stop (void);
void timer_function (union sigval sigval);
int kbhit(void);
int getch(void);

GDATALOGGER gDataLogger;

int timer_nr;
timer_t timer;
double Tglobal;
long int counter = 0;

volatile double tempo = 0.0;


int main (void)
{       
	int n;
#if ENABLE_SPEAKER_TEST
	timestruct_t timestruct;
	double texec;
#endif //ENABLE_SPEAKER_TEST

	// Permissão para acessar I/O
#if ENABLE_SPEAKER_TEST
	if(iopl(3)!=0){
		printf("\nErro em ioperm: ");
		int errsv = errno;
		switch(errsv){
		case EINVAL:
			printf("level is greater than 3");
			break;
		case ENOSYS:
			printf("level is greater than 3.");
			break;
		case EPERM:
			printf("The calling process has insufficient privilege to call iopl(); the CAP_SYS_RAWIO capability is required.");
			break;
		}
		return EXIT_FAILURE;
	}
	outb_p(inb_p(SPEAKER_CNTRL) & 0xFE, SPEAKER_CNTRL);

	time_reset(&timestruct);
	outb_p(inb_p(SPEAKER_CNTRL) ^ 0x02, SPEAKER_CNTRL);
	texec = time_gettime(&timestruct);

	printf("\n Tempo decorrido em teste de E/S: %f us",1e6*texec); 
#endif //ENABLE_SPEAKER_TEST

	// Data logger:
	if(!gDataLogger_Init(&gDataLogger,"matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return EXIT_FAILURE;
	}

	gDataLogger_DeclareVariable(&gDataLogger,"t","s",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"y1","m",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"y2","m",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"y3","m",1,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"MatA","s",3,1,1000);
	gDataLogger_DeclareVariable(&gDataLogger,"MatB","m",4,2,1000);

	// La�o principal
    timer_start ();

	n = 0;
	while(!kbhit()){
		usleep(20000);
		gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
		if(++n>10){
			gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
			printf("\n Tempo decorrido: %f s. Tglobal = %f ms. counter = %li",tempo,Tglobal*1000,counter);
			n = 0;
		}
	}
	printf("\n");

    timer_stop ();

	// Encerramento do data logger:
	gDataLogger_Close(&gDataLogger);

    return EXIT_SUCCESS;
}

void timer_start (void)
{
    struct itimerspec itimer = { { 1, 0 }, { 1, 0 } };
    struct sigevent sigev;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=TASK_PERIOD_US * 1000; 
    itimer.it_value=itimer.it_interval;

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = timer_nr;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = timer_function;

    if (timer_create (CLOCK_REALTIME, &sigev, &timer) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }

    if (timer_settime (timer, 0, &itimer, NULL) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

void timer_stop (void)
{
    if (timer_delete (timer) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

void timer_function (union sigval sigval)
{ 
	static int flagfirstexecution = 1,i,j;
	double T,y1,y2,y3;
	static timestruct_t timestruct;

	// Calculo das variaveis
	if(flagfirstexecution){
		flagfirstexecution = 0;
		T = 0.0;
	} else {
		T = time_gettime(&timestruct);
	}
	time_reset(&timestruct);
	tempo += T;
	y1 = tempo;
	y2 = sin(2*3.1415926*tempo);
	y3 = cos(2*3.1415926*tempo);
	Tglobal = T;

	MatA[0][0] = y1;
	MatA[1][0] = y2;
	MatA[2][0] = y3;

	for (i=0;i<4;++i){
		for (j=0;j<2;++j){
			MatB[i][j] = i*j;
		}
	}
	++counter;

	// Inserir na fila
	gDataLogger_InsertVariable(&gDataLogger,"t",(double *)&tempo);
	gDataLogger_InsertVariable(&gDataLogger,"y1",&y1);
	gDataLogger_InsertVariable(&gDataLogger,"y2",&y2);
	gDataLogger_InsertVariable(&gDataLogger,"y3",&y3);
	gDataLogger_InsertVariable(&gDataLogger,"MatA",&MatA[0][0]);
	gDataLogger_InsertVariable(&gDataLogger,"MatB",&MatB[0][0]);

	// Teste com alto-falante do PC: gera uma onda quadrada com periodo igual ao dobro do periodo da tarefa
#if ENABLE_SPEAKER_TEST
//	outb(inb(SPEAKER_CNTRL) ^ 0x02, SPEAKER_CNTRL);
	outb_p(inb_p(SPEAKER_CNTRL) ^ 0x02, SPEAKER_CNTRL);
#endif //ENABLE_SPEAKER_TEST
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
