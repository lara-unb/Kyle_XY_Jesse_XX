clear all;
close all;

mex -v -DDATALOGGER_COMPILE_FOR_CMEX=1 LDFLAGS="\$LDFLAGS -w -lrt -lm" -I../../ gdatalogger_ipc_retrievevariable.c ../../gdatalogger.c ../../gqueue.c ../../gmatlabdatafile.c;