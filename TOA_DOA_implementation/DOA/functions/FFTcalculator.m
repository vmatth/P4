function [xdata,ydata] = FFTcalculator(inputArray,fs)

    %t=linspace(0,length(inputArray)/fs,length(inputArray));
    Nfft=1024; %power of 2 and I put a huge number so there are many data points
    
    f=linspace(0,fs,Nfft);
    X1=abs(fft(inputArray,Nfft));

xdata = f(1:Nfft/2);
ydata = X1(1:Nfft/2);
end

