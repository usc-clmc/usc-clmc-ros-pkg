%compute mel filterbank
%B. Plannerer, May 2003

%clear all;
%close all;

%parameters
%-----------------------------------------
% fs = 8000; %sampling frequency
% N = 256; %FFT Window length
% nofChannels = 22; %number of mel channels
fs = 44100; %sampling frequency
N = 1024; %FFT Window length
nofChannels = 10; %number of mel channels
%-----------------------------------------

%compute resolution etc.
df = fs/N; %frequency increment on linear scale
Nmax = N/2; % maximum fft index
fmax = fs/2; % maximum frequency
melmax = freq2mel(fmax); % maximum mel frequency
melinc = melmax / (nofChannels + 1); % frequency increment on mel scale

%center frequencies on mel scale
melcenters = (1:nofChannels) .* melinc;

%center frequencies in linear scale [Hz]
fcenters = mel2freq(melcenters);

%compute filter bandwidths (linear scale)
startfreq = [0 , fcenters(1:(nofChannels-1))];
endfreq = [fcenters(2:nofChannels) , fmax];
bandwidth = endfreq - startfreq ;

%quantize into FFT indices
indexcenter = round(fcenters ./df);
%compute resulting frequencies
fftfreq = indexcenter.*df;

%compute start indices of windows
indexstart = [1 , indexcenter(1:nofChannels-1)];
%compute stop indices of windows
indexstop = [indexcenter(2:nofChannels),Nmax];
%compute bandwidth (number of indices per window)
idxbw = (indexstop - indexstart)+1;
%compute bendwidth [Hz]
FFTbandwidth = idxbw.*df;

%compute resulting quantization error
diff = fcenters - fftfreq;


%compute triangle-shaped filter coefficients
W = zeros(nofChannels,Nmax);
for c = 1:nofChannels
    %left ramp
    increment = 1.0/(indexcenter(c) - indexstart(c));
    for i = indexstart(c):indexcenter(c)
        W(c,i) = (i - indexstart(c))*increment;
    end
    %right ramp
    decrement = 1.0/(indexstop(c) - indexcenter(c));
    for i = indexcenter(c):indexstop(c)
       W(c,i) = 1.0 - ((i - indexcenter(c))*decrement);
    end
end

%save matrix
%save 'melmatrix' W;

%plot results

figure(5)

%plot filter frequencies
subplot(2,3,1);
%figure;
plot(fcenters,melcenters,'-o');
title('mel filter frequencies');
xlabel('f[Hz]');
ylabel('f[mel]');
grid on;

%plot channel numbers and center frequency FFT indices
%figure;
subplot(2,3,2);
plot(0:nofChannels-1,indexcenter,'-o');
title('FFT indices');
axis([0,nofChannels-1,-Inf,Inf]);
xlabel('filter channel');
ylabel('FFT index');
grid on;

%plot frequency quantization error
%figure;
subplot(2,3,3);
plot(0:nofChannels-1,(fcenters-fftfreq),'-o');
title('frequency quantization error');
axis([0,nofChannels-1,-Inf,Inf]);
xlabel('filter channel');
ylabel('f_{center} - f_{fft} [Hz]');
grid on;


%plot filter bandwidth
subplot(2,3,4)
plot(0:nofChannels-1,bandwidth,'-o');
axis([0,nofChannels-1,-Inf,Inf]);
title('filter bandwidth');
xlabel('filter channel');
ylabel('bandwidth [Hz]');
grid on;

%plot indexbandwidth
subplot(2,3,5);
plot(0:nofChannels-1,idxbw,'-o');
title('FFT filter bandwidth [indices]');
axis([0,nofChannels-1,-Inf,Inf]);
xlabel('filter channel');
ylabel('No. of FFT indices');
grid on;

%plot bandwidth error
subplot(2,3,6);
plot(0:nofChannels-1,(bandwidth - FFTbandwidth),'-o');
title('bandwidth quantization error');
axis([0,nofChannels-1,-Inf,Inf]);
xlabel('filter channel');
ylabel('bandwidth error [Hz]');
grid on;

figure(6)

%plot filter coefficients W
figure;
imagesc(1-W);
grid on;
title('matrix of filter coefficients');
xlabel('FFT index');
ylabel('filter channel');
colormap(gray);
