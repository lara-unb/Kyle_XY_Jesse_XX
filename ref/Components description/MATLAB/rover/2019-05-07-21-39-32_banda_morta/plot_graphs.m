[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 72.738792;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftout = wheelsvelocity.fieldleft_wheels;

plot(t,leftout)
hold on;

[n,p] = size(inpvelocity);
t = 1:n;
t = (t/n) * 72.738792;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftin = inpvelocity.fieldleft_wheels;
plot(t,leftin)

