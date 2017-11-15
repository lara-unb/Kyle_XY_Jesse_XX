clear all;
close all;

% LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/matlab/bin/glnx86:/usr/local/matlab/sys/os/glnx86:/usr/xenomai/lib
% export LD_LIBRARY_PATH 

% colocar aqui o resultado de /usr/xenomai/bin/xeno-config --xeno-cflags
cflags = 'CFLAGS="\$CFLAGS -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__"';
cflags = 'CFLAGS=" -D_GNU_SOURCE -fPIC -pthread -m32  -fexceptions -D_FILE_OFFSET_BITS=64 -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__"';

% colocar aqui o resultado de /usr/xenomai/bin/xeno-config --xeno-ldflags
% ldflags = 'LDFLAGS="\$LDFLAGS -L/usr/xenomai/lib -lpthread"';

% colocar aqui o resultado de /usr/xenomai/bin/xeno-config --skin=xenomai --ldflags
%ldflags = 'LDFLAGS="\$LDFLAGS -L/usr/xenomai/lib -lxenomai -lnative -lpthread"';
ldflags = 'LDFLAGS="\$LDFLAGS -L/usr/xenomai/lib -lpthread"';

% command = ['mex -v ',cflags,' ',ldflags,' -L/usr/xenomai/lib -lm -DDATALOGGER_COMPILE_FOR_XENOMAI=1 -DDATALOGGER_COMPILE_FOR_CMEX=1 -I../../ gdatalogger_ipc_retrievevariable.c ../../gdatalogger.c ../../gqueue.c ../../gmatlabdatafile.c /usr/xenomai/lib/libnative.a'];
command = ['mex -v ',cflags,' ',ldflags,' -L/usr/xenomai/lib -lm -DDATALOGGER_COMPILE_FOR_XENOMAI=1 -DDATALOGGER_COMPILE_FOR_CMEX=1 -I../../ gdatalogger_ipc_retrievevariable.c ../../gdatalogger.c ../../gqueue.c ../../gmatlabdatafile.c /usr/xenomai/lib/libnative.a /usr/xenomai/lib/libxenomai.a'];

eval(command);