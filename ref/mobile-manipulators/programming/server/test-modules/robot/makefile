ifeq ($(BUILDENV),)
#	$(>>> Para compilação pthread, execute make usando 'make clean all BUILDENV=linux-pthread' <<<)
#	echo ">>> Para compilação xenomai, execute make usando 'make clean all BUILDENV=linux-xenomai' <<<"
#	BUILDENV = linux-pthread
	BUILDENV = linux-xenomai
endif

ifeq ($(BUILDENV),linux-pthread)
	CFLAGS = -Wall -D_REENTRANT -D_POSIX_TIMERS
	CC = gcc
	LDFLAGS =  -lrt -lm
endif

ifeq ($(BUILDENV),linux-xenomai)

	XENO ?= /usr/xenomai

	XENOCONFIG=$(shell PATH=$(XENO):$(XENO)/bin:$(PATH) which xeno-config 2>/dev/null)

	### Sanity check
	ifeq ($(XENOCONFIG),)
	all::
		@echo ">>> Invoke make like this: \"make XENO=/path/to/xeno-config\" <<<"
		@echo
	endif

	CFLAGS=$(shell $(XENOCONFIG) --skin=native --cflags) -Wall  $(MY_CFLAGS)
	CC=$(shell $(XENOCONFIG) --cc)
	LDFLAGS=$(shell $(XENOCONFIG) --skin=native --ldflags) -lnative
	# This includes the library path of given Xenomai into the binary to make live
	# easier for beginners if Xenomai's libs are not in any default search path.
	LDFLAGS+=-Xlinker -rpath -Xlinker $(shell $(XENOCONFIG) --libdir)
endif

TARGET = test

INCLUDES = -I../../../commondefs/ -I../../

OBJS =  ${TARGET}.o
SRCS  =  main.c
SRCS +=  ../../keyboard.c
SRCS +=  ../../robot.c

ifeq ($(BUILDENV),linux-xenomai)
	INCLUDES +=  -I../../libs/serialcom-xenomai/src
	SRCS += ../../libs/serialcom-xenomai/src/serialcom.c 

	HDRS =  
	DEFS = -DLPROTOCOL_DEFINE_ALL=1 -DSERIALCOM_USE_DIRECT_IO_MODE=1
endif

ifeq ($(BUILDENV),linux-pthread)
	INCLUDES +=  -I../../libs/serialcom-directio 
	SRCS += ../../libs/serialcom-directio/serialcom.c 

	HDRS =  
	DEFS = -DLPROTOCOL_DEFINE_ALL=1 -DSERIALCOM_USE_DIRECT_IO_MODE=1 -DDATALOGGER_COMPILE_FOR_XENOMAI=0
endif


all: clean ${TARGET}

clean:
	rm ${TARGET} -f


# The variable $@ has the value of the target. In this case $@ = psort
${TARGET}:
	${CC} ${CFLAGS} ${INCLUDES} ${DEFS} ${SRCS} -o $@ ${LDFLAGS}



