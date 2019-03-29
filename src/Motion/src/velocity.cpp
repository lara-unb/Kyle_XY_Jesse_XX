/*-----------------------------------------------------------------------------------------------------/
/										                                                 /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição:                                                                              /
/-----------------------------------------------------------------------------------------------------*/

// Bibliotecas
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
//#include <signal.h>
#include <csignal>
#include <sys/io.h>
#include <sys/time.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <sys/select.h>
#include "sensoray526.h"
#include "SSC.h"
#include <iostream>


// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init)           \
    if (cmd_init == 0)                       \
    {                                        \
        printf("    Erro em %s", #cmd_init); \
        return (0);                          \
    }
#define MAIN_MODULE_CLOSE(cmd_close)          \
    if (cmd_close == 0)                       \
    {                                         \
        printf("    Erro em %s", #cmd_close); \
    }

// Definições para usar tic e toc pra cálculos de tempo
//---------------------------------------------------------------------------------------------------------
struct
{
    struct timeval time;
    struct timeval timereset;
} tictocctrl;

void tic(void)
{
    gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
    gettimeofday(&tictocctrl.time, NULL);
    return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec) * 1e-6);
}

//---------------------------------------------------------------------------------------------------------

void catch_signal(int sig)
{
}

void signalHandler( int signum ) {
   std::cout << "Interrupt signal (" << signum << ") received.\n";

   printf("\n*** Encerrando o modulo sensoray526...");
    MAIN_MODULE_CLOSE(sensoray526_close());

    printf("\n\n");
    fflush(stdout); // mostra todos printfs pendentes.

   exit(signum);  
}
//
//---------------------------------------------------------------------------------------------------------

//Lê os encoder por mais ou menos 1s
int readEncoder(void)
{
    float texec;
    unsigned char counter = 0;
    //Configura encoder 0
    sensoray526_configure_encoder(0);
    sensoray526_configure_encoder(1);
    sensoray526_reset_counter(0);
    sensoray526_reset_counter(1);
    tic();
    for (counter = 0; counter < 10; counter++)
    {
        //Le o encoder 0
        printf("\n Encoder 0: (%ld)", sensoray526_read_counter(0));
        printf("\n Encoder 1: (%ld)", sensoray526_read_counter(1));

        // Sleep
        usleep(100000);
    }
    texec = toc(); //us
    printf("\nRead encoder took %f seconds to execute \n", texec * 1e6);
    return 1;
}
//---------------------------------------------------------------------------------------------------------

// Calcula a velocidade das rodas em rad/s
void computeVel(void)
{
    float texec, t_now, tic0;
    unsigned char counter = 0;
    long n0 = 0;
    long n1 = 0;
    double w0, w1;
    clock_t t0 = clock();

    sensoray526_configure_encoder(0);
    sensoray526_configure_encoder(1);

    while(1)
    {
        sensoray526_reset_counter(0);
        sensoray526_reset_counter(1);
        tic();
        usleep(200000);
        n0 = sensoray526_read_counter(0); //n de ciclos encoder 0
        n1 = sensoray526_read_counter(1); //n de ciclos encoder 1
        texec = toc();
        t_now = ((clock() - t0) * 1000 )/CLOCKS_PER_SEC;  
        //w = (2*pi*num_of_cilcos)/(resolução_do_encoder* redução_do_motor*tempo)
        w0 = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952  
        w1 = (0.0020943952 * n1) / texec;
        printf("%lf, %ld, %ld, %lf, %lf\n", t_now, n0, n1, w0, w1);
    }
    
}
//---------------------------------------------------------------------------------------------------------

int main()
{ 
    signal(SIGTERM, catch_signal);
    signal(SIGINT, signalHandler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // robot module:
    printf("\n*** Iniciando o modulo sensoray526...");
    MAIN_MODULE_INIT(sensoray526_init());

    printf("Tempo (ms), Ciclos Enc 1, Ciclos Enc 2, Velocidade Enc 1 (rad/s), Velocidade Enc 2 (rad/s)\n");
    while (1)
    {
        computeVel();
    }
    return 0;
}