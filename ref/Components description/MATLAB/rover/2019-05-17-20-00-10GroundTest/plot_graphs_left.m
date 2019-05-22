clear;
inpvelocity = readtable('inp_velocity.csv');
wheelsvelocity = readtable('wheels_velocity.csv');
[n,p] = size(inpvelocity);
running_time = 40.914296;
t = 1:n;
t = (t/n) * running_time; 
x = inpvelocity.field_left_wheels;
y = wheelsvelocity.field_left_wheels;

for i = 1:n
    if(x(i) == 0)
        x(i) = 4;
    end
end

plot(t,x)
hold on;
plot(t,y)
