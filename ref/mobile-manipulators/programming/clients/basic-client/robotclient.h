/*****************************************************************************
*** Arquivo: robotclient.h
*** Conteudo: modulo robotclient.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 11-01-2011: criacao
*****************************************************************************/
/*! \file robotclient.h 
* \brief Arquivo cabecalho do modulo robotclient. */
#ifndef ROBOTCLIENT_H
#define ROBOTCLIENT_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:

// Prototipos de uso externo:
int robotclient_init(char *serveripnumber);
int robotclient_close(void);
int robotclient_get_time(float *ptime);
int robotclient_set_servos_move(servomovecommand_t *pservomovecommand);


#ifdef __cplusplus
}
#endif 

#endif // ROBOTCLIENT_H
