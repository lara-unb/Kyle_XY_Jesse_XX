hold on;
inpvelocity = readtable('inp_velocity.csv');
wheelsvelocity = readtable('wheels_velocity.csv');
[n,p] = size(wheelsvelocity);
running_time = 55.808594;
t = 1:n;
t = (t/n) * running_time;

x = inpvelocity.field_right_wheels;
y = wheelsvelocity.field_right_wheels;
for i = 1:n
    if(x(i) == 0)
        x(i) = 4;
    end
end

plot(t,x)
hold on;
plot(t,y)

%[n,p] = size(inpvelocity);
%t = 1:n;
%t = (t/n) * running_time;  
%leftin = inpvelocity.fieldleft_wheels;
%plot(t,leftin)
%rightout = rightout*(-1);
%[n,p] = size(wheelsvelocity);
%subplot(2,1,2)
%rightout = wheelsvelocity.fieldright_wheels;
%plot(t,rightout)
%hold on;
%plot(t,leftin)