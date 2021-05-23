function [arr] = squareWave(fs, samples,frequency)
% generates a square wave with specified length, frequency and sampling
% frequency
totalTime= samples/fs;
t = 0:1/fs:totalTime;
arr = square(2*pi*frequency*t)';
arr(1)=[];
t(1)=[];

end

