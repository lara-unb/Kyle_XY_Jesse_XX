/*****************************************************************************
*** Arquivo: timertask.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file timertask.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */

#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	#include <sys/mman.h>
	#include <native/task.h>
	#include <native/timer.h>
#endif

// Cabecalhos especificos do modulo:
#include "timertask.h"

// Definicoes internas:
#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
typedef struct{
		int realtimetaskperiodus;
		ptimertaskhandler_t ptimertaskhandler;
} xenomaitaskarg_t;

#else
	
#endif

// Prototipos internos:
#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	void timertask_xenomaitask(void *arg);
#else
	
#endif

// Variaveis do modulo:
static struct timeval timereset;

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/
/*! \fn int timertask_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int timertask_init(void)
{
 	gettimeofday(&timereset, NULL);
 	
 	return 1;
}


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI

	void timertask_start (timertaskcontrol_t *ptimertaskcontrol, ptimertaskhandler_t ptimertaskhandler, char *uniquetaskname, int periodus, int stacksize, int priority)
	{
		xenomaitaskarg_t xenomaitaskarg;

		/* Avoids memory swapping for this program */
        mlockall(MCL_CURRENT|MCL_FUTURE);

        /*
         * Arguments: &task,
         *            name,
         *            stack size (0=default),
         *            priority,
         *            mode (FPU, start suspended, ...)
         */
        rt_task_create(&ptimertaskcontrol->task_descriptor, uniquetaskname, stacksize, priority, 0);
        
		xenomaitaskarg.realtimetaskperiodus = periodus;
		xenomaitaskarg.ptimertaskhandler = ptimertaskhandler;
        /*
         * Arguments: &task,
         *            task function,
         *            function argument
         */
        rt_task_start(&ptimertaskcontrol->task_descriptor, &timertask_xenomaitask, &xenomaitaskarg);
	}

	void timertask_kill (timertaskcontrol_t *ptimertaskcontrol)
	{
		rt_task_delete(&ptimertaskcontrol->task_descriptor);
	}
#else
	void timertask_task(union sigval sigval)
	{
		timertask_function();
	}

	void timertask_start (void)
	{
		struct itimerspec itimer = { { 1, 0 }, { 1, 0 } };
		struct sigevent sigev;

		flag_firstexecution = 1;

		itimer.it_interval.tv_sec=0;
		itimer.it_interval.tv_nsec=realtimetaskperiodus * 1000; 
		itimer.it_value=itimer.it_interval;

		memset (&sigev, 0, sizeof (struct sigevent));
		sigev.sigev_value.sival_int = timertask_nr;
		sigev.sigev_notify = SIGEV_THREAD;
		sigev.sigev_notify_attributes = NULL;
		sigev.sigev_notify_function = timertask_task;

		if (timertask_create (CLOCK_REALTIME, &sigev, &timer) < 0)
		{
			fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
			exit (errno);
		}

		if (timertask_settime (timer, 0, &itimer, NULL) < 0)
		{
			fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
			exit (errno);
		}
	}

	void timertask_stop (void)
	{
		if (timertask_delete (timer) < 0)
		{
			fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
			exit (errno);
		}
	}
#endif

double timertask_gettime(void)
{
	struct timeval time;

	gettimeofday(&time, NULL);
	return ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec)*1e-6);
}

/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	void timertask_xenomaitask(void *arg)
	{
		ptimertaskhandler_t ptimertaskhandler;
		int realtimetaskperiodus;
		
		
		ptimertaskhandler = ((xenomaitaskarg_t *)(arg))->ptimertaskhandler;
		realtimetaskperiodus = ((xenomaitaskarg_t *)(arg))->realtimetaskperiodus;
		
		rt_task_set_periodic(NULL, TM_NOW, realtimetaskperiodus * 1000);

		while (1) {
			rt_task_wait_period(NULL);
			if(ptimertaskhandler!=NULL){
				if(!ptimertaskhandler()){
					return;
				}
			}
		}
	}
	
#else
	
#endif
