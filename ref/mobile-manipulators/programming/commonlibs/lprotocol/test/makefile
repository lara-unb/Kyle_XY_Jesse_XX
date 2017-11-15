# gcc -Wall -D_REENTRANT -D_POSIX_TIMERS timer.c -o timer -lrt 
CFLAGS = -Wall -D_REENTRANT -D_POSIX_TIMERS -DLPROTOCOL_DEFINE_ALL=1

TARGET = test
CC = gcc
LIBS =  -lpthread 
INCLUDES =
OBJS =  ${TARGET}.o
SRCS =  ${TARGET}.c ../lprotocol.c
HDRS =  

clean:
	rm ${TARGET} -f

all: clean ${TARGET}

# The variable $@ has the value of the target. In this case $@ = psort
${TARGET}: 
	${CC} ${CFLAGS} ${INCLUDES}  ${SRCS} -o $@ ${LIBS}



