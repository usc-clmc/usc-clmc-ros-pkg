%clear all;
%close all;

N = 10;
Fs = 44100;

x = sin(2*pi*110*(0:(1/Fs):5));

size(x)
plot(x)

WindowLength = 1024;
[Pxx,F] = periodogram(x,[],WindowLength,Fs);

Filter = melfilter2(N,F);
Filter = melfilter2(N,F,@rectwin);
[Filter,MF] = melfilter2(N,F,@blackmanharris);

FPxx = Filter*Pxx;

%keyboard

%plot filter coefficients W
figure;
imagesc(1-Filter);
grid on;
title('matrix of filter coefficients');
xlabel('FFT index');
ylabel('filter channel');mel
colormap(gray); 

size(Filter)