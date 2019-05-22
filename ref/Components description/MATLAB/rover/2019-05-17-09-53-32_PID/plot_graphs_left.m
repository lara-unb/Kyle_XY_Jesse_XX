clear;
inpvelocity = readtable('inp_velocity.csv');
wheelsvelocity = readtable('wheels_velocity.csv');
[n,p] = size(inpvelocity);
running_time = 23.646527;
t = 1:n;
t = (t/n) * running_time; 
x = inpvelocity.field_left_wheels;
y = wheelsvelocity.field_left_wheels;

plot(t,x)
hold on;
plot(t,y)
