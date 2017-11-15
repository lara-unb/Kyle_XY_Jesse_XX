/*****************************************************************************
*** Arquivo: sensoray526.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 18-01-2011: criacao
*****************************************************************************/
/*! \file sensoray526.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <stdio.h>
#include <math.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */

#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

// Cabecalhos especificos do modulo:
#include "sensoray526.h"

// Definicoes internas:
#define S526_IOSIZE		0x40  /* 64 bytes */

// Prototipos internos:

// Variaveis do modulo:
int sensoray526baseaddress = 0x0100;

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int sensoray526_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int sensoray526_init(void)
{
	// Inicializa variaveis globais, aloca memoria, etc.
	if(iopl(3)<0){
		printf("\n sensoray526_init: iopl error");
		return 0;
	}
	
/*	if (!request_region(sensoray526baseaddress, S526_IOSIZE, "s526")) {
		printf("\n sensoray526_init: I/O port conflict");
		return 0;
	}	
*/
	// Guarda configurao.

	// Inicializa threads.

	// Retorna 
	return 1; 
}                      

/*! \fn int sensoray526_close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/

int sensoray526_close(void)
{
	// Procedimentos de encerramento
	
	// Retorna 
   	return 1; 
}                      


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
int sensoray526_set_dio(unsigned char value, unsigned char mask)
{
	outw_p(((value & mask) & 0xFF) | (1<<10) | (1<<11), sensoray526baseaddress + S526_REG_DIO);
	
	return 1;
}

unsigned char sensoray526_get_dio(unsigned char mask)
{
	return ((inw_p(sensoray526baseaddress + S526_REG_DIO) & mask) & 0xFF);
}

void sensoray526_write_register(unsigned int value16bits, int registeroffset)
{
	outw_p(value16bits, sensoray526baseaddress + registeroffset);
}

unsigned int sensoray526_read_register(int registeroffset)
{
	return inw_p(sensoray526baseaddress + registeroffset) & 0xFFFF;
}

/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

