for i = 2:n
    if((abs(y(i))>=4*abs(y(i-1))) & (abs(y(i-1))>1))
        y(i) = y(i-1);
    end
end

plot(t,x)
hold on;
plot(t,y)
