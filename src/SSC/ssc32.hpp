/* Universidade de Brasília
 * Laboratório de Automação e Robótica
 * Autor: Gabriel Guimarães Almeida de Castro
 * Programa: Básico da comunicação serial e movimentação dos servos
 */

//Include headers
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sstream>
#include <cstdio>

// Define COM port
#define COM_PORT            "/dev/ttyUSB0"

// Define Baud Rate
#define BAUDRATE            B115200

// Define Port Timeout
#define TIMEOUT_SEC         10

namespace Servo
{
	/***************
	*   SSC-32 servo controller library
	*   
	*****************/
	class SSC32
	{
	    private:  
   	        int fd; // file description for the serial port
            struct termios port_settings;
            struct timeval timeout;
            std::string command;
            std::string Port, Position, Speed;
            const char *Output;
            int numberOfPorts;
            double servoLimits[32][6];
	    public:
	        SSC32();
	        ~SSC32();
   	        void initSerial();
   	        bool startSerial(std::string);
   	        void stopSerial();
            bool SendMessage(int Port, int position, int speed);
            bool CheckIfFinished();
	};
};//namespace   