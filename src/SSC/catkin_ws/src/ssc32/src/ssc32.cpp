#include "ssc/ssc32.h"

namespace ssc
{

//Constructor
	SSC32::SSC32( ) :
		fd( -1 )
	{
		fd = openPort();
		setServos();
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

	int SSC32::verifyPositionBounds()
	{
		for (int i = 0; i < NUM_SERVOS; ++i)
		{
			if (pos[i] > MAX_PULSE_WIDTH or pos[i] < MIN_PULSE_WIDTH)
  			{
  				std::cout << "Position is not within its boundaries --- \n" << pos[i] << "\n";
    			return 0;
  			}
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

	void SSC32::moveServo()
	{
		if(verifyPositionBounds() == 1)
 		{
 			int n;
 			std::string str;
 			for (int i = 0; i < NUM_SERVOS; ++i)
 			{	
 				if(i == 0)
 					str = '#' + intToString(i) + " P" + intToString(pos[i]) + " S" + intToString(speed[i]);
 				if(pos[i] != 0)
 					str += '#' + intToString(i) + " P" + intToString(pos[i]) + " S" + intToString(speed[i]);
 			}
 			str += '\r';
    		const char * msg = str.c_str();
    		n = write(fd, msg, strlen(msg));
    		if (n < 0)
      		fputs("write() failed!\n", stderr);
  		}

	}

	void SSC32::setServos()
	{
		for (int i = 0; i < NUM_SERVOS; ++i)
		{
			pos.push_back(CENTER_PULSE_WIDTH);
		}
    	moveServo();
    	sleep(0.1); //give each servo initial wait time
  
	}

	int SSC32::getNUM_SERVOS()
	{
		return this->NUM_SERVOS;
	}

	void SSC32::setPosSpeed(int Arr0[], int Arr1[])
	{
		for (int i = 0; i < NUM_SERVOS; ++i)
		{
			pos[i] = Arr0[i];
			speed[i] = Arr1[i];
		}
	}

}//namespace
