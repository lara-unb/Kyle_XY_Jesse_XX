/*****************************************************************************
*** Arquivo: module.h
*** Conteudo: modulo module.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- DATADECRIACAO: criacao
*****************************************************************************/
/*! \file module.h 
* \brief Arquivo cabecalho do modulo module. */
#ifndef MODULE_H
#define MODULE_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:

// Prototipos de uso externo:
int module_init(void);
int module_close(void);

#ifdef __cplusplus
}
#endif 

#endif // MODULE_H
