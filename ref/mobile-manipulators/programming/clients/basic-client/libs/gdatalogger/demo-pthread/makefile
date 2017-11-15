# gcc -Wall -D_REENTRANT -D_POSIX_TIMERS timer.c -o timer -lrt 
CFLAGS = -Wall -D_REENTRANT -D_POSIX_TIMERS -DDATALOGGER_COMPILE_FOR_XENOMAI=0

TARGET = demo_pthread
CC = gcc
LIBS =  -lrt -lm
INCLUDES =
OBJS =  ${TARGET}.o
SRCS =  ${TARGET}.c ../gdatalogger.c ../gmatlabdatafile.c  ../gqueue.c
HDRS =  

clean:
	rm ${TARGET} -f

all: clean ${TARGET}

# The variable $@ has the value of the target. In this case $@ = psort
${TARGET}:
	${CC} ${CFLAGS} ${INCLUDES}  ${SRCS} -o $@ ${LIBS}



