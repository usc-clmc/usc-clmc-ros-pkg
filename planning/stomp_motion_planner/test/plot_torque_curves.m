clear all;
close all;

figSize = [3 2];
inset = 0.02;
figPos = [inset inset figSize(1)-inset*2 figSize(2)-inset*2];

torques=load('torques.txt');
t_normal = mean(torques(1:10,:));
t_normal_std = std(torques(1:10,:));
t_normal_plus = t_normal + t_normal_std;
t_normal_minus = t_normal - t_normal_std;
t_opt = mean(torques(11:20,:));
t_opt_std = std(torques(11:20,:));
t_opt_plus = t_opt + t_opt_std;
t_opt_minus = t_opt - t_opt_std;

time=5*[0.01:0.01:0.99];

figure(1);
axis([0.05 4.9 32 58]);
hold on;
%h_normal = fill([time'; flipud(time')], [t_normal_plus'; flipud(t_normal_minus')], [0.8 0.8 0.8]);
%h_opt = fill([time'; flipud(time')], [t_opt_plus'; flipud(t_opt_minus')], [0.8 0.8 0.8]);
l_normal = plot(time, t_normal, ...);
        'LineWidth'     , 2, ...
        'LineStyle'     , '-.', ...
        'Color'         , [1 0 0] ...
);
l_opt = plot(time, t_opt, ...);
        'LineWidth'     , 2, ...
        'Color'         , [0 0 1] ...
);
%line = plot(cmean, ...
%        'LineWidth'     , 2, ...
%        'Color'         , [0 0 1] ...
%);
leg = legend([l_normal l_opt], {'No torque opt.', 'Torque opt.'}, 'Location', 'North');
xl=xlabel('Time (sec)');
yl=ylabel('Sum of abs. joint torques (Nm)');
set(gca,'box','on');
for i=[gca xl yl leg]
        set(i,'FontSize',10);
        set(i,'FontName','Times');
end
set(gcf,'PaperUnits','inches');
set(gcf,'PaperSize',figSize);
set(gcf,'PaperPosition', figPos);
print -painters -dpdf -r300 torques.pdf
