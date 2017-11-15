clear all;
close all;

Filename = 'gmatlabdatafile.mat';

t  = ReadGMatlabDataFile('t', Filename);
y1 = ReadGMatlabDataFile('y1', Filename);
y2 = ReadGMatlabDataFile('y2', Filename);
y3 = ReadGMatlabDataFile('y3', Filename);

MatA = ReadGMatlabDataFile('MatA', Filename);
MatB = ReadGMatlabDataFile('MatB', Filename);

%N = min([size(t) size(texec) size(y1)]);

figure; 
subplot(311), plot(t,y1); title('Variavel y1');
subplot(312), plot(t,y2); title('Variavel y2');
subplot(313), plot(t,y3); title('Variavel y3');

figure; 
plot(diff(t)); title('Periodo de amostragem');

figure;
for i=1:length(t)    x(i) = MatA(1,1,i); end
subplot(311), plot(t,x); title('Variavel y1 (MatA)');
for i=1:length(t)    x(i) = MatA(2,1,i); end
subplot(312), plot(t,x); title('Variavel y2 (MatA)');
for i=1:length(t)    x(i) = MatA(3,1,i); end
subplot(313), plot(t,x); title('Variavel y3 (MatA)');

whos
