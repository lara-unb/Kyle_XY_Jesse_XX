[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 72.061388;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftout = wheelsvelocity.fieldleft_wheels;

plot(t,leftout)
hold on;

[n,p] = size(inpvelocity);
t = 1:n;
t = (t/n) * 72.061388;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftin = inpvelocity.fieldleft_wheels;
plot(t,leftin)

plot
