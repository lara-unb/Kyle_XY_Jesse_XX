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
#include <stdlib.h>
#include <sstream>
#include <vector>

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
			const static unsigned int NUM_SERVOS = 6;
			std::vector<int> pos;

			SSC32( );
			~SSC32( );

			int openPort(void);
			bool is_connected( );
			void moveServo();
			void closePort();
			int getNUM_SERVOS();
			void setPos(int Arr[]);

		private:
			std::string intToString(int n);
			int verifyPositionBounds();
			void setServos();
	};
}

#endif