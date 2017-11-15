/*****************************************************************************
*** Arquivo: keyboard.h
*** Conteudo: modulo keyboard.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file keyboard.h 
* \brief Arquivo cabecalho do modulo keyboard. */
#ifndef KEYBOARD_H
#define KEYBOARD_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:

// Prototipos de uso externo:
int kbhit(void);
int getch(void);

#ifdef __cplusplus
}
#endif 

#endif // KEYBOARD_H
