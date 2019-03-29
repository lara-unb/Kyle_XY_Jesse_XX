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
#include <signal.h>
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

//
//---------------------------------------------------------------------------------------------------------
int kbhit(void)
{
    struct timeval tv;
    fd_set read_fd;

    /* Do not wait at all, not even a microsecond */
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    /* Must be done first to initialize read_fd */
    FD_ZERO(&read_fd);

    /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
    FD_SET(0, &read_fd);

    /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
    if (select(1, &read_fd, NULL, /*No writes*/ NULL, /*No exceptions*/ &tv) == -1)
        return 0; /* An error occured */

    /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */

    if (FD_ISSET(0, &read_fd))
        /* Character pending on stdin */
        return 1;

    /* no characters were pending */
    return 0;
}
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
    float texec;
    unsigned char counter = 0;
    long n0 = 0;
    long n1 = 0;
    double w;

    sensoray526_configure_encoder(0);
    sensoray526_configure_encoder(1);
    sensoray526_reset_counter(0);
    sensoray526_reset_counter(1);

    tic();

    // Sleep
    usleep(100000);

    n0 = sensoray526_read_counter(0); //n of pulses encoder 0
    n1 = sensoray526_read_counter(1); //n of pulses encoder 1

    texec = toc();
    w = (2 * 3.14159275 * n0) / (100 * 30 * texec);
    printf("\n Speed: %f rad/s", w);
}
//---------------------------------------------------------------------------------------------------------

int main()
{
    int mode = 0;

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // robot module:
    printf("\n*** Iniciando o modulo sensoray526...");
    MAIN_MODULE_INIT(sensoray526_init());

    while (1)
    {
        computeVel();
    }

    printf("\n*** Encerrando o modulo sensoray526...");
    MAIN_MODULE_CLOSE(sensoray526_close());

    printf("\n\n");
    fflush(stdout); // mostra todos printfs pendentes.
    return 1;
}
