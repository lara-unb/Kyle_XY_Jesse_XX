/*****************************************************************************
*** Arquivo: ..c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file ..cpp
* \brief Arquivo exemplo de modulo. */

// Cabecalhos des biblioteca padrao C:
#include <math.h>
#include <stdio.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */

// Cabecalhos especificos do modulo:
#include "..h"

// Definicoes internas:

// Prototipos internos:

// Variaveis do modulo:

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int ._init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int ._init(void)
{
	// Inicializa variaveis globais, aloca memoria, etc.

	// Guarda configurao.

	// Inicializa threads.

	// Retorna 
	return 1; 
}                      

/*! \fn int ._close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/

int ._close(void)
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

