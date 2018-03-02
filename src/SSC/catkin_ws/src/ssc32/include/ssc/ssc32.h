/* Universidade de Brasília
 * Laboratório de Automação e Robótica
 * Autor: Gabriel Guimarães Almeida de Castro
 * Descrição: Básico da comunicação serial e movimentação dos servos
 */

#ifndef SSC_SSC32_H
#define SSC_SSC32_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <iostream>
#include <string> 
#include <stdlib.h>
#include <sstream>

namespace ssc
{

	class SSC32
	{
		public:
			int fd;
			const static unsigned int MAX_PULSE_WIDTH =	2500;
			const static unsigned int CENTER_PULSE_WIDTH =	1500;
			const static unsigned int MIN_PULSE_WIDTH =	500;
			const static unsigned int MAX_CHANNELS = 32;

			SSC32( );
			~SSC32( );

			int openPort(void);
			bool is_connected( );
			void moveServo(int ch, int pos);
			void closePort();

		private:
			std::string intToString(int n);
			int verifyBounds(int ch, int pos);
			void setServos(int num);
	};
}

#endif