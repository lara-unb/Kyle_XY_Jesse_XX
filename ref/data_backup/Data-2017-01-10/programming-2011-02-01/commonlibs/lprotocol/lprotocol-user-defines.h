/*******************************************************************************
* \file lprotocol-user-defines.h
* \brief  Deixa aqui definições do usuário.
// Observaçoes:
*******************************************************************************/

#ifndef LPROTOCOLUSERDEFINES_H
#define LPROTOCOLUSERDEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo quer dependem da aplicação do usuario:
#define LPROCOTOL_MAX_FUNCTIONS									20

#define LPROCOTOL_FUNCTION_ACK									0
#define LPROCOTOL_FUNCTION_ERROR_FUNCTION_UNKNOWN				1
#define LPROCOTOL_FUNCTION_ERROR_WRONG_NUMBER_OF_ARGUMENTS		2
#define LPROCOTOL_FUNCTION_GETTIME								3
#define LPROCOTOL_FUNCTION_GETANGLEPOTENTIOMETERREADINGS		4
#define LPROCOTOL_FUNCTION_GETFOOTINFRAREDREADINGS				5
#define LPROCOTOL_FUNCTION_GETFOOTGYROREADINGS					6
#define LPROCOTOL_FUNCTION_GETMOTORCURRENTREADINGS				7
#define LPROCOTOL_FUNCTION_SETMODE								8
#define LPROCOTOL_FUNCTION_SETALLMOTORPWM						9
#define LPROCOTOL_FUNCTION_SETKNEEMOTORPWM						10
#define LPROCOTOL_FUNCTION_SETANKLESAGITALMOTORPWM				11
#define LPROCOTOL_FUNCTION_SETANKLEFRONTALMOTORPWM				12

#ifdef __cplusplus
}
#endif 

#endif // LPROTOCOLUSERDEFINES_H


