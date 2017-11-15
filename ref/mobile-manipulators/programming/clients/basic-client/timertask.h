/*****************************************************************************
*** Arquivo: timertask.h
*** Conteudo: modulo timertask.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file timertask.h 
* \brief Arquivo cabecalho do modulo timertask. */
#ifndef TIMERTASK_H
#define TIMERTASK_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
#if MMROBOTCLIENT_COMPILE_FOR_XENOMAI
	typedef struct{
		RT_TASK 		task_descriptor;
	} timertaskcontrol_t;
#else
	typedef struct{
	} timertaskcontrol_t;
#endif

typedef int (*ptimertaskhandler_t)();

// Prototipos de uso externo:
int timertask_init(void);
void timertask_start (timertaskcontrol_t *ptimertaskcontrol, ptimertaskhandler_t ptimertaskhandler, char *uniquetaskname, int periodus, int stacksize, int priority);
void timertask_kill (timertaskcontrol_t *ptimertaskcontrol);
double timertask_gettime(void);

#ifdef __cplusplus
}
#endif 

#endif // TIMERTASK_H
