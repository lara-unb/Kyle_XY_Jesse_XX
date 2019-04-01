/*****************************************************************************************************
*** Arquivo: ssc32_kj_arms
*** Conteudo: Biblioteca SSC32 
*** Modificado por: Felipe Luis Rodrigues Sousa
*** Código fonte: https://github.com/mnovo/lynxmotion_ros/blob/master/include/lynxmotion_ssc32/ssc32.h
******************************************************************************************************/

#ifndef LYNX_MOTION_SSC32_SSC32_KJ_ARMS_H
#define LYNX_MOTION_SSC32_SSC32_KJ_ARMS_H

#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace lynxmotion_ssc32
{

class SSC32
{
	public:
			const static unsigned int MAX_PULSE_WIDTH = 2500;
			const static unsigned int CENTER_PULSE_WIDTH = 1500;
			const static unsigned int MIN_PULSE_WIDTH = 500;

			const static unsigned int MAX_CHANNELS = 32;

			struct ServoCommand
			{
				unsigned int ch;
				unsigned int pw;
				int spd;

				ServoCommand(): ch(0), pw(SSC32::CENTER_PULSE_WIDTH), spd(-1) {}
			};

			SSC32();
			~SSC32();

			bool open_port(const char *port, int baud);
			bool is_connected();
			void close_port();
			bool move_servo(struct ServoCommand cmd, int time = -1);
			bool move_servo(struct ServoCommand cmd[], unsigned int n, int time = -1);
			bool cancel_command();
			bool pulse_offset(unsigned int ch, int value);
			bool pulse_offset(unsigned int ch[], int value[], unsigned int n);

			enum LogicLevel
			{
				Low,	// 0V
				High	// +5V
			};

			bool discrete_output(unsigned int ch, LogicLevel lv1);
			bool discrete_output(unsigned int ch[], LogicLevel lv1[], unsigned int n);

			/*
			* Este comando permite que 8 bits de dados sejam gravados de uma só vez.
			* Todos os pinos do bank são atualizados simultaneamente. Os banks ão atualizados dentro de 20ms ao receber a mensagem.
			* Parametro bank 0 = Pins 0-7, 1 = 8-15, etc...
			* (Como estaremos trabalhando com apenas 6 pinos, será necesśario apenas o bank 0)
			* Parametro value eh o valor decimal de saida para o bank selecionado (0-255). Bit 0 = LSB do bank.
			* Retorna true se a mensagem foi enviada com secesso; cc. false.
			*/

			bool byte_output(unsigned int bank, unsigned int value);

			/*
			* Retorna true se algum servo estiver em movimento, cc. false.
			*/

			bool query_movement_status();

			/*
			* Retorna a largura de pulso do servo selecionado (500 - 2500)
			*/

			int query_pulse_width(unsigned int ch);



			enum Inputs
			{
				PinA,
				PinAL,
				PinB,
				PinBL,
				PinC,
				PinCL,
				PinD,
				PinDL,
				PinE,	// Somente SSC-32U
				PinEL,	// Somente SSC-32U
				PinF,	// Somente SSC-32U
				PinFL,	// Somente SSC-32U
				PinG,	// Somente SSC-32U
				PinH,	// Somente SSC-32U
			};



			bool read_digital_inputs(Inputs inputs[], unsigned int outputs[], unsigned int n);

			bool read_analog_inputs(Inputs inputs[], float outputs[], unsigned int n);

			std::string get_version();
	


	private:

			/*
			*	Envia uma mensagem para o SSC32.
			*	Parametro msg eh a mensagem a sendo enviada ao servo controlador.
			*	Parametro size eh o tamanho da mensagem enviada.
			*	
			*	Retorna true se não houver erros durante o envio da mensagem, cc. false.
			*/

			bool send_message(const char *msg, int size);

			unsigned int recv_message(unsigned char *buf, unsigned int size);

			int fd; // Descritor de arquivo para a porta serial
			int first_instruction[32];
};

}// namespace

#endif