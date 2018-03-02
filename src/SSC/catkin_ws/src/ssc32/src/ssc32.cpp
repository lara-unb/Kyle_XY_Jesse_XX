#include "ssc/ssc32.h"

namespace ssc
{

//Constructor
	SSC32::SSC32( ) :
		fd( -1 )
	{
		fd = openPort();
		setServos(6);
	}

	//Destructor
	SSC32::~SSC32( )
	{
		closePort( );
	}

	int SSC32::openPort(void)
	{
		int fd1; /* File descriptor for the port */
  		struct termios options;
  		int baud = B115200;

		fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd1 < 0)
		{
    		perror("openPort: Unable to open /dev/ttyUSB0\n"); //It coud not open port
		}
  		else
    		fcntl(fd1, F_SETFL, 0);

		memset(&options, 0, sizeof(options));
		cfsetispeed( &options, baud );
		cfsetospeed( &options, baud );

		options.c_iflag = IGNBRK | IGNPAR;
		options.c_oflag = 0;
		options.c_cflag |= CREAD | CS8 | CLOCAL;
		options.c_lflag = 0;

		if( tcsetattr( fd1, TCSANOW, &options ) < 0 )
		{
    		printf( "ERROR: setting termios options\n" );
    		return false;
    	}

  		// Make sure queues are empty
  		tcflush( fd1, TCIOFLUSH );

		printf( "Successfully opened port /dev/ttyUSB0\n");

		return (fd1);
	}

	std::string SSC32::intToString(int n)
	{
		std::ostringstream str1;
		str1 << n;
		std::string str = str1.str();
		return str1.str();
	}

	int SSC32::verifyBounds(int ch, int pos)
	{
		if (ch >= MAX_CHANNELS or ch < 0)
		{
    		std::cout << "Channel is not within its boundaries\n";
    		return 0;
  		}
  		if (pos > MAX_PULSE_WIDTH or pos < MIN_PULSE_WIDTH)
  		{
  			std::cout << "Position is not within its boundaries\n";
    		return 0;
  		}
  		return 1;
	}

	bool SSC32::is_connected( )
	{
		return ( fd != -1 );
	}

	void SSC32::closePort()
	{
		close(fd);
	}

	void SSC32::moveServo(int ch, int pos)
	{
		if(verifyBounds(ch, pos) == 1)
 		{
    		int n;
    		std::string str = '#' + intToString(ch) + " P" + intToString(pos) + '\r';
    		const char * msg = str.c_str();
    		n = write(fd, msg, strlen(msg));
    		if (n < 0)
      		fputs("write() failed!\n", stderr);
  		}

	}

	void SSC32::setServos(int num)
	{
		int NUM_ACTIVE_SERVOS = num;

  		for(int i = 1; i <= NUM_ACTIVE_SERVOS; i++) {
    	moveServo(i-1, 1500);
    	sleep(0.1); //give each servo initial wait time
  		}
	}


}//namespace
