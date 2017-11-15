/*****************************************************************************
*** Arquivo: robotserver.h
*** Conteudo: modulo robotserver.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file robotserver.h 
* \brief Arquivo cabecalho do modulo robotserver. */
#ifndef ROBOTSERVER_H
#define ROBOTSERVER_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:

// Prototipos de uso externo:
int robotserver_init(void);
int robotserver_close(void);

#ifdef __cplusplus
}
#endif 

#endif // ROBOTSERVER_H
