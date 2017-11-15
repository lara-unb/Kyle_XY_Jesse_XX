function main

close all;

disp('Iniciando dataviewer...');

% t = ReadGMatlabDataFile('t',Filename);		%% time
% T_exec = ReadGMatlabDataFile('T_exec',Filename);		%% execution time
% gyro_1 = ReadGMatlabDataFile('g_1',Filename);	%% gyro 1
% gyro_2 = ReadGMatlabDataFile('g_2',Filename);	%% gyro 2
% i_1 = ReadGMatlabDataFile('i_k',Filename);	%% motor current knee
% i_2 = ReadGMatlabDataFile('i_as',Filename);	%% motor current ankle sagital
% i_3 = ReadGMatlabDataFile('i_af',Filename);	%% motor current ankle frontal
% u_1 = ReadGMatlabDataFile('u_k',Filename);	%% motor pwm input [0-1000]	- knee
% u_2 = ReadGMatlabDataFile('u_as',Filename);	%% motor pwm input [0-1000]	- ankle sagital
% u_3 = ReadGMatlabDataFile('u_af',Filename);	%% motor pwm input [0-1000]	- ankle frontal
% theta_1 = ReadGMatlabDataFile('p_k',Filename);	%% knee joint potentiometer
% theta_2 = ReadGMatlabDataFile('p_as',Filename);	%% knee joint potentiometer
% theta_3 = ReadGMatlabDataFile('p_af',Filename);	%% knee joint potentiometer
% r_1 = ReadGMatlabDataFile('r_1',Filename);	%% foot infrared range 1
% r_2 = ReadGMatlabDataFile('r_2',Filename);	%% foot infrared range 2
% r_3 = ReadGMatlabDataFile('r_3',Filename);	%% foot infrared range 3
% r_4 = ReadGMatlabDataFile('r_4',Filename);	%% foot infrared range 4
% 
% u_1 = 12*(u_1 - 500)/500;
% u_2 = 12*(u_2 - 500)/500;
% u_3 = 12*(u_3 - 500)/500;
% 
% i_1 = (i_1*3.3/1023 - 2.5)/0.133;
% i_2 = (i_2*3.3/1023 - 2.5)/0.133;
% i_3 = (i_3*3.3/1023 - 2.5)/0.133;

t = [];
T_exec = [];
theta_k = [];
theta_as = [];
theta_af = [];
i_k = [];
i_as = [];
i_af = [];
u_k = [];
u_as = [];
u_af = [];
gyro_1 = [];
gyro_2 = [];
r_1 = [];
r_2 = [];
r_3 = [];
r_4 = [];

twindow = 10;
status = 0;

while(status~=4)
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('p_k');
    [theta_k] = [theta_k; data]; theta_k_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('p_as');
    [theta_as] = [theta_as; data]; theta_as_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('p_af');
    [theta_af] = [theta_af; data]; theta_af_unitstring = dataunitstring; 

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('g_1');
    [gyro_1] = [gyro_1; data]; gyro_1_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('g_2');
    [gyro_2] = [gyro_2; data]; gyro_2_unitstring = dataunitstring;

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('r_1');
    [r_1] = [r_1; data]; r_1_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('r_2');
    [r_2] = [r_2; data]; r_2_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('r_3');
    [r_3] = [r_3; data]; r_3_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('r_4');
    [r_4] = [r_4; data]; r_4_unitstring = dataunitstring;

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('i_k'); 
    data = (data*3.3/1023 - 2.5)/0.133;
    [i_k] = [i_k; data]; i_k_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('i_as');
    data = (data*3.3/1023 - 2.5)/0.133;
    [i_as] = [i_as; data]; i_as_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('i_af');
    data = (data*3.3/1023 - 2.5)/0.133;
    [i_af] = [i_af; data]; i_af_unitstring = dataunitstring; 

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('u_k'); 
    data = 12*(data - 500)/500;
    [u_k] = [u_k; data]; u_k_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('u_as');
    data = 12*(data - 500)/500;
    [u_as] = [u_as; data]; u_as_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('u_af');
    data = 12*(data - 500)/500;
    [u_af] = [u_af; data]; u_af_unitstring = dataunitstring; 

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('T_exec');
    [T_exec] = [T_exec; data]; T_exec_unitstring = dataunitstring; 

    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('t');
    [t]  = [t ; data]; t_unitstring  = dataunitstring;
    
    if(status~=4)
        figure(1);
        subplot(311); myplot(t,t_unitstring,theta_k,'\theta_k',theta_k_unitstring,'b',twindow);
        subplot(312); myplot(t,t_unitstring,theta_as,'\theta_{as}',theta_as_unitstring,'b',twindow);
        subplot(313); myplot(t,t_unitstring,theta_af,'\theta_af',theta_af_unitstring,'b',twindow);

        figure(2);
        subplot(311); myplot(t,t_unitstring,i_k, 'i_k',i_k_unitstring,'b',twindow);
        subplot(312); myplot(t,t_unitstring,i_as,'i_{as}',i_as_unitstring,'b',twindow);
        subplot(313); myplot(t,t_unitstring,i_af,'i_{af}',i_af_unitstring,'b',twindow);

        figure(3);
        subplot(311); myplot(t,t_unitstring,u_k, 'u_k',u_k_unitstring,'b',twindow);
        subplot(312); myplot(t,t_unitstring,u_as,'u_{as}',u_as_unitstring,'b',twindow);
        subplot(313); myplot(t,t_unitstring,u_af,'u_{af}',u_af_unitstring,'b',twindow);

        figure(4);
        subplot(211); myplot(t,t_unitstring,gyro_1,'gyro_1',gyro_1_unitstring,'b',twindow);
        subplot(212); myplot(t,t_unitstring,gyro_2,'gyro_2',gyro_2_unitstring,'b',twindow);

        figure(5);
        subplot(411); myplot(t,t_unitstring,r_1,'r_1',r_1_unitstring,'b',twindow);
        subplot(412); myplot(t,t_unitstring,r_2,'r_2',r_2_unitstring,'b',twindow);
        subplot(413); myplot(t,t_unitstring,r_3,'r_3',r_3_unitstring,'b',twindow);
        subplot(414); myplot(t,t_unitstring,r_4,'r_4',r_4_unitstring,'b',twindow);
        
        figure(6); 
        myplot(t,t_unitstring,[0;diff(t)],'T_s, T_{exec}','s','r',twindow); hold on;
        myplot(t,t_unitstring,T_exec,'T_s, T_{exec}','s','b',twindow); hold on;
        title('Período de amostragem (vermelho) e período de execução da tarefa periódica (azul)');        
        
        drawnow;
        pause(0.1);
    end
end

disp('Saindo...');

return;

function myplot(t,tunit,var,varname,varunit,color,twindow)

kmax = min([length(t),length(var)]);

if(t(kmax)>twindow)
    taxismin = t(kmax) - twindow;
    taxismax = t(kmax);
else
    taxismin = 0;
    taxismax = twindow;
end
plot(t(1:kmax),var(1:kmax),color); 
ax = axis; axis([taxismin taxismax ax(3) ax(4)]);
xlabel(sprintf('t [%s]',tunit)); 
ylabel(sprintf('%s [%s]',varname,varunit)); 

return;
