[n,p] = size(wheelsvelocity);
t = 1:n;
t = (t/n) * 72.946883; % 30.787933 is the time running rosbag
%leftin = inppwm.fieldleft_wheels;
%leftout = wheelsvelocity.fieldleft_wheels;

%plot(t,leftout*25+1528)
%hold on;
%plot(t+0.2,leftin)


rightin = inppwm.fieldright_wheels;
rightout = wheelsvelocity.fieldright_wheels;

plot(t,rightout*25+1532)
hold on;
plot(t+0.2,rightin)


