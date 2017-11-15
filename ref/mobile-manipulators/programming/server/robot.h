/*****************************************************************************
*** Arquivo: robot.h
*** Conteudo: modulo robot.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 12-01-2011: criacao
*****************************************************************************/
/*! \file robot.h 
* \brief Arquivo cabecalho do modulo robot. */
#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:

// Prototipos de uso externo:
int robot_init(void);
int robot_close(void);
int robot_get_time(float *ptime);
int robot_set_servos_move(servomovecommand_t *pservomovecommand);

#ifdef __cplusplus
}
#endif 

#endif // ROBOT_H
