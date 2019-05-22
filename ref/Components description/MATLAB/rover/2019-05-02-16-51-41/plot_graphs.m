[n,p] = size(pidvelocity);
t = 1:n;
t = (t/n) * 36.897561;  % 36.897561 is the time running rosbag
leftInp = pidvelocity.fieldleft_wheels;
leftInp = wheelsvelocity.fieldright_wheels;

plot(t,leftInp)
hold on;
plot(t,leftPid)
hold on;

