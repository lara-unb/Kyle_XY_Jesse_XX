%[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 1.000215117874225e+02;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
%leftout = -1*wheelsvelocity.fieldleft_wheels;
subplot(2,1,1)
plot(t,leftout)
hold on;

%[n,p] = size(inpvelocity);
t = 1:n;
t = (t/n) * 1.000215117874225e+02;  % 30.787933 is the time running rosbag
%leftin = inpvelocity.fieldleft_wheels;
plot(t,leftin)
%rightout = rightout*(-1);
%[n,p] = size(wheelsvelocity);
subplot(2,1,2)
%rightout = wheelsvelocity.fieldright_wheels;
plot(t,rightout)
hold on;
plot(t,leftin)