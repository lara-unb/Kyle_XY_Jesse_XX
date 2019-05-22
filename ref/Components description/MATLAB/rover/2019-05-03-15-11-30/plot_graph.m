[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 74.834666;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftout = wheelsvelocity.fieldleft_wheels;

plot(t,leftout)
hold on;

[n,p] = size(inpvelocity);
t = 1:n;
t = (t/n) * 30.787933;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftin = inpvelocity.fieldleft_wheels;
plot(t,leftin)

