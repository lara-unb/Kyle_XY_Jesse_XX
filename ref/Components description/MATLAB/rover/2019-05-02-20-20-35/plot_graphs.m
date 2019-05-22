[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 36.897561;  % 36.897561 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftout = wheelsvelocity.fieldright_wheels;

plot(t,leftout)


