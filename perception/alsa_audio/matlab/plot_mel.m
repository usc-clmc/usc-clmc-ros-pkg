function plot_mel()

load /tmp/mel.txt;
%load /tmp/f_c.txt;
%load /tmp/phi_c.txt;

%figure(124)
%plot(f_c)
%title('f_c')

%figure(41241)
%plot(phi_c)

%plot filter coefficients W
figure(2320002);

size(mel)

mel = mel';

size(mel)

imagesc(1-mel);
grid on;
title('matrix of filter coefficients');
xlabel('FFT index');
ylabel('filter channel');
colormap(gray);