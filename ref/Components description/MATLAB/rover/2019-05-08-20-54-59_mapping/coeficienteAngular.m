%coeficiente angular da reta

%time = time - 1.557359707505026e+18;
t = 12.47e+9;
i = 1;
while(time(i) < t)
    i =  i + 1;
    display(i)
end
k = 1;
while(t < 22.45e+9)
    m(i) = (pwm(i+1) - pwm(i))/((time(i+1) - time(i))*10^-9);
    t = time(i+1);
    i = i + 1;
end

media = mean(m);