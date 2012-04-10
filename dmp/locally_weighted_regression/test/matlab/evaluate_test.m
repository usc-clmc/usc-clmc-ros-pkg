clear all;
close all;

basis_function_matrix = load('../data/basis_function_matrix.txt');

% input x
test_x = load('../data/test_x.txt');
% target y
test_y = load('../data/test_y.txt');

% querry
test_xq = load('../data/test_xq.txt');

% prediction
test_yp = load('../data/test_yp.txt');

figure(1)

subplot(2, 1, 1)

hold on;
box on;
plot(test_x, test_y, 'xb');
plot(test_xq, test_yp, 'r');
hold off;

subplot(2, 1, 2)

hold on;
box on;
plot(basis_function_matrix)
hold off;

