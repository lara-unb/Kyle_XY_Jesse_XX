clear;
inpvelocity = readtable('inp_velocity.csv');
wheelsvelocity = readtable('wheels_velocity.csv');
[n,p] = size(wheelsvelocity);
running_time = 73.681429;
t = 1:n;
t = (t/n) * running_time; 
x = inpvelocity.field_left_wheels;
y = wheelsvelocity.field_left_wheels;

plot(t,x)
hold on;
plot(t,y)
