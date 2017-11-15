function main

close all;

disp('Iniciando dataviewer...');

t = [];
y1 = [];
y2 = [];
y3 = [];

figure(1);
subplot(311); h_y1 = plot(0,0); set(h_y1,'EraseMode','xor');
subplot(312); h_y2 = plot(0,0); set(h_y2,'EraseMode','xor');
subplot(313); h_y3 = plot(0,0); set(h_y3,'EraseMode','xor');

figure(2); 
h_T = plot(0,0); set(h_T,'EraseMode','xor'); title('Período de amostragem');        

twindow = 10;
status = 0;
while(status~=4)
    tic
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('y1');
    toc
    [y1] = [y1; data]; y1_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('y2');
    [y2] = [y2; data]; y2_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('y3');
    [y3] = [y3; data]; y3_unitstring = dataunitstring;
    [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable('t');
    [t]  = [t ; data]; t_unitstring  = dataunitstring;

    if(status~=4)
        figure(1);
        subplot(311); myplot(h_y1,t,t_unitstring,y1,'y1',y1_unitstring,'b',twindow); ax1 = axis;
        subplot(312); myplot(h_y2,t,t_unitstring,y2,'y2',y2_unitstring,'b',twindow); ax2 = axis;
        subplot(313); myplot(h_y3,t,t_unitstring,y3,'y3',y3_unitstring,'b',twindow); ax3 = axis;
        
        ax1 = [min([ax1(1) ax2(1) ax3(1)]) min([ax1(2) ax2(2) ax3(2)]) ax1(3) ax1(4)];
        ax2 = [min([ax1(1) ax2(1) ax3(1)]) min([ax1(2) ax2(2) ax3(2)]) ax2(3) ax2(4)];
        ax3 = [min([ax1(1) ax2(1) ax3(1)]) min([ax1(2) ax2(2) ax3(2)]) ax3(3) ax3(4)];

        subplot(311); axis(ax1);
        subplot(312); axis(ax2);
        subplot(313); axis(ax3);

        figure(2); 
        set(h_T,'XData',t,'YData',[0;diff(t)]); ax = axis; axis([min(t) max(t) ax(3) ax(4)]);
%        plot(t,[0;diff(t)]);  
        
        drawnow;
        pause(0.1);
    end
end

disp('Saindo...');

return;

function myplot(h,t,tunit,var,varname,varunit,color,twindow)

kmax = min([length(t),length(var)]);
if(t(kmax)>twindow)
    taxismin = t(kmax) - twindow;
    taxismax = t(kmax);
else
    taxismin = 0;
    taxismax = twindow;
end
set(h,'XData',t(1:kmax),'YData',var(1:kmax),'Color',color); 
ax = axis; axis([taxismin taxismax ax(3) ax(4)]);
xlabel(sprintf('t [%s]',tunit)); 
ylabel(sprintf('%s [%s]',varname,varunit)); 

return;
