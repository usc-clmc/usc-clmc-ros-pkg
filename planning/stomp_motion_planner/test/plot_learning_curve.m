clear all;
close all;

figSize = [3 2];
inset = 0.02;
figPos = [inset inset figSize(1)-inset*2 figSize(2)-inset*2];

seed = 7;
trajs = zeros(1,10);
for i=1:5
  trajs(i*2-1) = (i-1)*42 + seed;
  trajs(i*2) = (i-1)*42 + seed + 1;
end

%trajs = [3 4 45 46 87 88 129 130 171 172]

curves=load('curves.txt');

curves=curves(trajs,:);
cmean = mean(curves);
cstd = std(curves);
cstdplus = cmean + cstd;
cstdminus = cmean - cstd;

axis([1 200 0 700]);
xs=[1:200]';
figure(1);
hold on;
h = fill([xs; flipud(xs)], [cstdplus'; flipud(cstdminus')], [0.8 0.8 0.8]);
line = plot(cmean, ...
        'LineWidth'     , 2, ...
        'Color'         , [0 0 1] ...
);
leg = legend([line h], {'Trajectory cost', '\pm 1 standard deviation'}, 'Location', 'NorthEast');
xl=xlabel('Iteration number');
yl=ylabel('Trajectory cost');
set(gca,'box','on');
for i=[gca xl yl leg]
        set(i,'FontSize',10);
        set(i,'FontName','Times');
end
set(gcf,'PaperUnits','inches');
set(gcf,'PaperSize',figSize);
set(gcf,'PaperPosition', figPos);
print -painters -dpdf -r300 curves.pdf
