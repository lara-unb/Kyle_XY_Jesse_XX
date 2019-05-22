[n,p] = size(data);
t = 1:n;
t = (t/n) * 19.993496;  % 19.993496 is the time running rosbag
left = data.fieldleft_wheels;
right = data.fieldright_wheels;


plot(t,left)
hold on;
plot(t,right)
