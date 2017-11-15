/*****************************************************************************
*** Arquivo: module.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- DATADECRIACAO: criacao
*****************************************************************************/
/*! \file module.cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */

// Cabecalhos especificos do modulo:
#include "module.h"

// Definicoes internas:

// Prototipos internos:

// Variaveis do modulo:

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int module_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int module_init(void)
{
	// Inicializa variaveis globais, aloca memoria, etc.

	// Guarda configurao.

	// Inicializa threads.

	// Retorna 
	return 1; 
}                      

/*! \fn int module_close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/

int module_close(void)
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


/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

