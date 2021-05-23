
clear;
close all;
clc;
for index = 1:100
%% TOA estimation using real Data
addpath(genpath('m_files'));
addpath(genpath('srcSignal'));
addpath(genpath('lib'));


setup=defaultMiscSetup([]);
setup=defaultSignalSetup(setup);
setup=defaultArraySetup(setup);
setup=defaultRirGenSetup(setup);
setup=defaultRoomSetup(setup);
setup=defaultEmSetup(setup);
setup = defaultNLSSetup(setup);

selectedFrequencies = 2^11;
setup.fftLength = selectedFrequencies;
setup.signal.nfft = selectedFrequencies;
srcFactor = 1;
thresholdGamma = 5e6;
snrGrid = 40;
mcIter = 1;
lookAng = 1:4:359;
toaInterval=round((2*[setup.EM.minimumDistance;setup.EM.maximumDistance]...
    -setup.array.micRadius)/setup.room.soundSpeed*setup.signal.sampFreq);

windowNdx=toaInterval(1):(toaInterval(2)+setup.signal.lengthBurst);
lengthWindow=length(windowNdx);

grids.toa=toaInterval(1):toaInterval(2);
%% Clear All Port


saveData = false;
plotEnable = false;
comportEnabled = false;
recordEnabled = false;
offset = 0;
trials = 1;

%% Generate signals used for the experiment

% Length of the sound burst
setup.signal.lengthBurst = 5000;
% length of the signal
setup.signal.lengthSignal = 20000;
% sampling frequency
sampleRate = 48000;
setup.signal.sampFreq = sampleRate;

% resample signal by factor
sampleFactor = 1;

%% Initializing playRec
deviceList=playrec('getDevices');

deviceID=17;
inputChannel=[1:7];
outputChannel=[1:2];
recordTime=1;
signalObservNoHwDelay = zeros(20000,length(inputChannel),trials);

deviceForPlayRec=deviceList(deviceID);

if playrec('isInitialised')
    playrec('reset');
end
playrec('init',sampleRate,deviceID,deviceID);

%% Source Signal
nwin = 440;
winbkman = blackman(nwin);
winbkman = winbkman(nwin/2+1:end);
rng default
% [bhp,ahp]=butter(4,4/24,'high');

% winSignal = filter(bhp,ahp,randn(setup.signal.lengthBurst,1));
winSignal = randn(setup.signal.lengthBurst,1);
winSignal(end-length(winbkman)+1:end) = winSignal(end-length(winbkman)+1:end).*winbkman;
winSignal = 0.9*winSignal/max(abs(winSignal));

soundPlayback=[winSignal;...
              zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];

          
%% Record Background Noise
ambientPage = playrec('rec', recordTime*sampleRate, inputChannel);

while playrec('isFinished',ambientPage)<1
end
disp('Ambient Noise Done recording!');
ambientNoise=playrec('getRec',ambientPage);
signals.bgNoise = ambientNoise;

costFunctionDelay1 = zeros(length(grids.toa),length(lookAng));
costFunctionDelay2 = zeros(length(grids.toa),length(lookAng));

%% Probe the environment
disp('Playing and recording sound...');
recordPage=playrec('playrec',srcFactor*soundPlayback*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);

while playrec('isFinished',recordPage)<1
end
disp('Done recording!');
recordData=playrec('getRec',recordPage);
    % Save the file in .wav format
    % audiowrite(['class_room_sound',datestr(now, 'dd-mmm-yyyy'),'.wav'],recordData,sampleRate);
    % save the file in .mat format
    % save(['class_room_sound',datestr(now, 'dd-mmm-yyyy'),'.mat']);
signalObserve = recordData;
    % Normalize recordData
%     recordData(:,1) = recordData(:,1)/max(recordData(:,1));

%% Remove Hardware delay due to Preamplifier
[pks, locs]=findpeaks(recordData(:,1),'MinPeakProminence',0.001);
rirEstFft = fft(recordData(locs(1):end,2:end), setup.signal.nfft)./fft(soundPlayback, setup.signal.nfft);
rirEst = ifft(rirEstFft);

%%%     filtered loudspeaker response
%     soundPlayback = filter(rirEst(81:200),1, soundPlayback);
signals.observ = recordData(locs(1)+offset:end,:);
signals.clean = soundPlayback;

ii= 1;
signalObservNoHwDelay(:,:,ii) = signals.observ(1:20000,:);
signals.observation = signalObservNoHwDelay;


%% Reconstruct RIR from received and observed signals
signals.signalObservFft = fft(signals.observ, setup.signal.nfft); % include direct path comp
% signals.signalObservReflFft = fft(signals.observRefl, setup.signal.nfft);
signals.signalCleanFft = fft(signals.clean, setup.signal.nfft);

tic
 
for mm=1:mcIter

%% signals
signals.observ = signalObservNoHwDelay(:,2:end);
signals.clean = soundPlayback;

[dist1Span, dist1, tau1, MaxIndex1] = DistanceCalc(signals.clean, signals.observ(:,1), setup,index, 1);
[dist2Span, dist2, tau2, MaxIndex2] = DistanceCalc(signals.clean, signals.observ(:,2), setup,index, 2); 
[dist3Span, dist3, tau3, MaxIndex3] = DistanceCalc(signals.clean, signals.observ(:,3), setup,index, 3); 
%[dist4Span, dist4, tau4, MaxIndex4] = DistanceCalc(signals.clean, signals.observ(:,4), setup,index, 4);
%[dist5Span, dist5, tau5, MaxIndex5] = DistanceCalc(signals.clean, signals.observ(:,5), setup,index, 5);
%[dist6Span, dist6, tau6, MaxIndex6] = DistanceCalc(signals.clean, signals.observ(:,6), setup,index, 6);
fileID = fopen('testingAtDifferentDistances_11-05-2021_test2.csv', 'a');
%fprintf(fileID, 'Testing at loudspeaker wall distance at: 60cm \n');
fprintf(fileID, '%4.2f; %4.2f; %4.2f \n',dist1,dist2,dist3);
fclose(fileID);
close all;
index

    
end


end