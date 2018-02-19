/* Universidade de Brasília
 * Laboratório de Automação e Robótica
 * Autor: Gabriel Guimarães Almeida de Castro
 * Programa: Básico da comunicação serial e movimentação dos servos
 */

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
int fd;

//Convert integer to string
std::string int_to_string(int n)
{
  std::ostringstream str1;
  str1 << n;
  std::string str = str1.str();
  return str1.str();
}

int open_port(void)
{
  int fd; /* File descriptor for the port */
  struct termios options;
  int baud = B115200;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
  {
    perror("open_port: Unable to open /dev/ttyUSB0\n"); //It coud not open port
  }
  else
    fcntl(fd, F_SETFL, 0);

  memset(&options, 0, sizeof(options));
  cfsetispeed( &options, baud );
  cfsetospeed( &options, baud );

  options.c_iflag = IGNBRK | IGNPAR;
  options.c_oflag = 0;
  options.c_cflag |= CREAD | CS8 | CLOCAL;
  options.c_lflag = 0;

  if( tcsetattr( fd, TCSANOW, &options ) < 0 )
  {
    printf( "ERROR: setting termios options\n" );
    return false;
  }

  // Make sure queues are empty
  tcflush( fd, TCIOFLUSH );

  printf( "Successfully opened port /dev/ttyUSB0\n");

  return (fd);
}

//move the ch servo to postition pos
void moveServo(int ch, int pos)
{
  int n;
  std::string str = '#' + int_to_string(ch) + " P" + int_to_string(pos) + '\r';
  const char * msg = str.c_str();
  n = write(fd, msg, strlen(msg));
  if (n < 0)
  fputs("write() failed!\n", stderr);
}

//centralizes all servos
void setServos(int num) {
  int NUM_ACTIVE_SERVOS = num;

  for(int i = 1; i <= NUM_ACTIVE_SERVOS; i++) {
    moveServo(i, 1500);
    sleep(0.1); //give each servo initial wait time
  }
}

int main()
{
  int fd1 = open_port();
  fd = fd1;
  setServos(6);
  moveServo(5,500);
  sleep(1);
  moveServo(5,1500);
  moveServo(4,2500);
  sleep(1);
  moveServo(4,500);
  close(fd);

}
