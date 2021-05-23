load echoSnr01_Dist0.88.mat
extracted = measurementData.audioData(:,1,1);


plot(extracted)

Probe = measurementData.probedSignal(:,1);
%figure(1)
subplot(2,1,1)
plot(extracted)
title('playback')
subplot(2,1,2)
plot(Probe)
title('Recorded')

%% udregn delay ud fra afstand
%t = s/v
dist_min = 30;
dist_max = 100;
distances = [dist_min:dist_max];
Tau = distances;
for i = 1:length(distances)
    Tau(i) = distances(i)/34300; %Delays for all distances - 34300 = speed of sound in cm/s
end    

%% Z-værdi
K = 1024; %length of FFT Aka S
Z = zeros(K,length(Tau));
for k = 1:length(Tau)
    for j = 1:K
        Z(j,k) = exp(-1i*Tau(k)*2*pi*((j-1)/j));
    end 
end
plot(real(Z))

%% 
