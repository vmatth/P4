function [distance, tau, MaxIndex] = DistanceCalc(playback, recorded, setup)
%% Load
recordedData = recorded; %Vi bruger kun noget af længden på signalet som er vigtig ergo 1:3000 (Kan måske give fejl hvis lyden ekko lyden er længere end 3000 samples...)
%playbackData = playback(1:setup.signal.lengthBurst,1); %Bruger kun den del af playbackData hvor der er lyd (resten er 0)
playbackData = playback;

%% Frequency Domain

%Known signal (Lyden vi sender ud)
s = playbackData; %Gemmer værdien over i 's' så det ligner matematikken fra deres rapport
S = fft(s, 2048); %PlaybackData in Frequency domain
%S = fft(s, setup.signal.nfft); %PlaybackData in Frequency domain

%Observed signal (Lyden vi modtager)
w_k = recordedData; %RecordedData
W_k = fft(w_k, 2048); %RecordedData in Frequency domain
%W_k = fft(w_k, setup.signal.nfft); %RecordedData in Frequency domain

%% Time Plot
% figure(1)
% subplot(2,1,1)
% plot(playbackData)
% title('Playback Time Domain')
% subplot(2,1,2)
% plot(recordedData)
% title('Recorded Time Domain')

%% FFT Plot
% fs = 48000;
% [x1,y1] = FFTcalculator(W_k, setup.signal.sampFreq);
% [x2,y2] = FFTcalculator(S, setup.signal.sampFreq);
% 
% figure(2)
% subplot(2,1,1);
% plot(x2, y2);
% title('Playback Frequency Domain');
% 
% subplot(2,1,2);
% plot(x1, y1);
% title('Recorded Frequency Domain');
%% Delay og tau estimat
%Vi opstiller et estimat som distancen kan være i
dist_min = 34; % i cm
dist_max = 300;
distances = linspace(dist_min,dist_max,1000);
%distances = (dist_min:dist_max);

%Vi omregner distance estimat til tid estimat i samples.
%Tau = TOA
Tau = 1:length(distances); %Laver en Tau matrixe med samme dimensioner som distances.
for i = 1:length(distances) %For hver distance værdi regner vi Tau 
    Tau(i) = round(distances(i)/(setup.room.soundSpeed*100)*setup.signal.sampFreq);
    %Distancen deles med lydens hastighed for at få det i tid (der ganges
    %med 100 fordi vores distance var i cm og ikke m)
    %Tiden ganges med sampFreq for at få det i samples
end    

Tau = Tau'; %Transpose (Bytter om på dimensionerne)

%% Z-værdi
%Vi laver Z matrixen som ses i Eq. (8)
%Der laves en Z for hver tau værdi

K = length(S); %length of FFT Aka S
Z = zeros(K,1); %One page of Z
%3D matrix (Each page is a new Z(t) 
%Z(:,:,1) is Z matrix for tau index 1
%Z(:,:,2) is Z matrix for tau with index 2 osv..

for k = 1:length(Tau) %k = antal side i 3D matrixen
    for j = 1:K %j = Hver kolonne i matrixen
        Z(j,1,k) = exp(-1i*Tau(k)*2*pi*((j-1)/K)); %Formel fra Eq (8)
    end 
end

%% Equation (12)
%Vi opstiller Z_bar og WH_k som i rapporten
Z_bar = Z.*S;
WH_k = W_k';

Answer = 1:length(Tau); %Ny matrixe til alle svar (Mener nok at Answer svarer til Costfunction)
for i = 1:length(Tau) %Vi regner answer ud for hver tau værdi.
    Answer(i) = real(WH_k * Z_bar(:,:,i));
end


%Der hvor cost funktionen er højest finder vi indekset. 
[MaxVal, MaxIndex] = max(Answer);
%Plot af answer
%   figure('Name','Cross-correlation');
%   plot(Answer);
%       hold on
%       plot(MaxIndex,MaxVal,'or');
%       legend('Cost function','Max value');
%       xlabel('Index number for tau value');
%       str = sprintf('Cross-correlation with SNR at %d', setup.signal.snr);
%       title(str);
    
    
%Index bruges til at finde distancen, ved at indsætte index i distance matrixen.

%Convert to distance
distance = (distances(MaxIndex)); %Distancen regnes ud ved at /2
tau = Tau(MaxIndex); %Tau printes også (Hvis man kigger på RIR plot, så vil denne tau værdi ligge der hvor den første reverberation kommer.
