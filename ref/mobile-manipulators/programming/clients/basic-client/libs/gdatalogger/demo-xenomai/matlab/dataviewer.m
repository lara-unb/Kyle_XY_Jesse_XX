function main

close all;

disp('Iniciando dataviewer...');

t1 = [];
t2 = [];
y1 = [];
y2 = [];

twindow = 10;
status = 0;

while(status~=4)
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('y1');
    [y1] = [y1; data]; y1_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('t1');
    [t1]  = [t1 ; data]; t1_unitstring  = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('y2');
    [y2] = [y2; data]; y2_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('t2');
    [t2]  = [t2 ; data]; t2_unitstring  = dataunitstring;
    
    if(status~=4)
        figure(1);
        subplot(211); myplot(t1,t1_unitstring,y1,'y1',y1_unitstring,'b',twindow);
        subplot(212); myplot(t2,t2_unitstring,y2,'y2',y2_unitstring,'b',twindow);
        
        figure(2); 
        subplot(211); plot(t1,[0;diff(t1)]); title('Período de amostragem da tarefa 1');        
        subplot(212); plot(t2,[0;diff(t2)]); title('Período de amostragem da tarefa 2');        
        
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
