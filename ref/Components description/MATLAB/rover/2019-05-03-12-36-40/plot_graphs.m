
[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 34.773266;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftout = wheelsvelocity.fieldleft_wheels;

plot(t,leftout)
hold on;

[n,p] = size(inpvelocity);
t = 1:n;
t = (t/n) * 34.773266;  % 30.787933 is the time running rosbag
%leftinp = inpvelocity.fieldleft_wheels;
leftin = inpvelocity.fieldleft_wheels;
plot(t,leftin)


media = 0.0;
for i = n:2
    media = media + (leftout(i) - leftout(i-1));
end
%media = media + leftout(1);
media = media / (n-1);


    
    