function eval_lwr_test()

close all;
clear all;

xq = load('test_xq.txt');
x = load('test_x.txt');
yp = load('test_yp.txt');
y = load('test_y.txt');

figure(1)
hold on;
box on;
plot(x,y,'o');
plot(xq, yp, 'rx');
hold off;
