clc;
clear all;
close all;

comPort = 'COM5';
baudRate = 115200;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

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
setup.fftLength = 2^11;
setup.signal.nfft = 2^11;
srcFactor = 1;
toaInterval=round((2*[setup.EM.minimumDistance;setup.EM.maximumDistance]...
    -setup.array.micRadius)/setup.room.soundSpeed*setup.signal.sampFreq);

windowNdx=toaInterval(1):(toaInterval(2)+setup.signal.lengthBurst);
lengthWindow=length(windowNdx);

grids.toa=toaInterval(1):toaInterval(2);

%% Clear All Port

directoryLoc = 'D:\AudioAnalysisLab\OneDrive - Aalborg Universitet\Aalborg University\OneDrive - Aalborg Universitet\Aalborg University\AudioAnalysisLab\13Nov2019\';
saveData = false;
plotEnable = true;
comportEnabled = false;
recordEnabled = true;
offset = 0;
% A script used to save audio data in .wav file as well as save .mat file
% This script is used to generate a Gaussian signal that 

% Generate signals used for the experiment

% Length of the sound burst
setup.signal.lengthBurst = 1500;
% length of the signal
setup.signal.lengthSignal = 5000;
% sampling frequency
sampleRate = 48000;
% resample signal by factor
sampleFactor = 1;

setup.signal.sampFreq = sampleRate;
%%
nwin = 440;
winbkman = blackman(nwin);
winbkman = winbkman(nwin/2+1:end);
rng default
% [bhp,ahp]=butter(4,4/24,'high');

% winSignal = filter(bhp,ahp,randn(setup.signal.lengthBurst,1));
winSignal = randn(setup.signal.lengthBurst,1);
winSignal(end-length(winbkman)+1:end) = winSignal(end-length(winbkman)+1:end).*winbkman;
winSignal = 0.9*winSignal/max(abs(winSignal));

%% logrithmic chirp

% t= 0:1/sampleRate:0.001;
% fo = 100;
% f1 = 20000;
% chirpSig = chirp(t,fo,0.001,f1,'quadratic', [],'concave');
% chirpSig = chirpSig';
% sound(chirpSig, samplingFreq)
% spectrogram(chirpSig,256,200,1000);

%% sweptSinusoid - From SMARD data
%%
% startFreq = 10000;
% stopFreq = 20000;
% samplingFreq = sampleRate;
% sweepTime = 0.02266;
% fadeOutTime = 0;
% sweepIsExponential = true;
% sweptSinusoidalSignal = sweptSinusoid(startFreq,stopFreq,samplingFreq,...
%                       sweepTime,fadeOutTime,sweepIsExponential);
% sound(sweptSinusoidalSignal,samplingFreq)
                  % plot(sweptSinusoidalSignal);
%% comport
if comportEnabled
    [serialInterface] = serial(comPort, 'BaudRate', baudRate);
    fopen(serialInterface);
    fprintf(serialInterface, '0');
end
%%

    tic
    %% stop the robot for 1 second
    if comportEnabled
%         fprintf(serialInterface, '8');
%         pause(1);
%         fprintf(serialInterface, '0');
    end
%     pause(2);

%%
    % Generating a random noise signal followed by a zero padding
%       soundPlayback=[sweptSinusoidalSignal;...
%               zeros(setup.signal.lengthSignal-length(sweptSinusoidalSignal),1)];
    %% 
%       soundPlayback=[chirpSig;...
%               zeros(setup.signal.lengthSignal-length(chirpSig),1)];
    %%
      soundPlayback=[winSignal;...
              zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];
%     
        % delay the input signal by 5000 samples.
    % To ensure that zero-padding in before and after signals
%     soundPlayback=delayseq(soundPlayback,5000);

    % [soundPlayback,sampleRate]=audioread('gaussian_white_noise.wav');
    deviceList=playrec('getDevices');

    deviceID=17;
    inputChannel=[1:1];
    outputChannel=[1:2];
    recordTime=1;
    
    deviceForPlayRec=deviceList(deviceID);

    if playrec('isInitialised')
        playrec('reset');
    end
    playrec('init',sampleRate,deviceID,deviceID);
    if recordEnabled

%     pause(3);
        disp('Playing and recording sound...');
    recordPage=playrec('playrec',(srcFactor^2)*soundPlayback*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);

    while playrec('isFinished',recordPage)<1
    end
    disp('Done recording!');
    recordDataOriginal=playrec('getRec',recordPage);
    pause(3);
        % record file


        ambientPage = playrec('rec', recordTime*sampleRate, inputChannel);

        while playrec('isFinished',ambientPage)<1
        end
        disp('Ambient Noise Done recording!');
        ambientNoise=playrec('getRec',ambientPage);
        
        % Normalized ambient Signal
%         ambientNoise = ambientNoise(:,1)/max(ambientNoise(:,1));
        
        % Calculating power of the ambient Noise
        pSNR = var(recordDataOriginal(:,1))/var(ambientNoise(:,1));
        snrdB = 10*log10(pSNR)
        % Play the file only using PLAY button
%         playrec('play',ambientNoise,outputChannel(1));
%         figure(010);
%         plot(ambientNoise);
    end
    pause(2);
% while 1
%%
for ii=1:1
    
%%
    disp('Playing and recording sound...');
    recordPage=playrec('playrec',(srcFactor^2)*soundPlayback*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);

    while playrec('isFinished',recordPage)<1
    end
    disp('Done recording!');
    recordData=playrec('getRec',recordPage);
    
    pSNR = var(recordData(:,1))/var(ambientNoise(:,1));
    snrdB = 10*log10(pSNR)
    % Save the file in .wav format
    % audiowrite(['class_room_sound',datestr(now, 'dd-mmm-yyyy'),'.wav'],recordData,sampleRate);
    % save the file in .mat format
    % save(['class_room_sound',datestr(now, 'dd-mmm-yyyy'),'.mat']);

    % Normalize recordData
%     recordData(:,1) = recordData(:,1)/max(recordData(:,1));
%%
    [pks, locs]=findpeaks(recordData(:,2),'MinPeakProminence',0.001);
    rirEstFft = fft(recordData(locs(1):end,1), setup.signal.nfft)./fft(soundPlayback, setup.signal.nfft);
    rirEst = ifft(rirEstFft);

%%     filtered loudspeaker response
%     soundPlayback = filter(rirEst(81:200),1, soundPlayback);

    
%%
    % play and plot recordDATA (OPTIONAL)
%     if plotEnable
%         figure(100)
%         subplot(311)
%         plot(recordData(locs(1)+offset:end,1))
%         subplot(312)
%         spectrogram(recordData((locs(1))+offset:end,1),hamming(length(recordData(locs(1):end,1))/100),round(0.75*(length(recordData(locs(1),1)))/100),2048, sampleRate, 'yaxis')
%         subplot(313)
%         plot(rirEst)
%     end
    %% Save data

%     pause(5);

signals.observ = recordData(locs(1)+offset:end,1);
signals.clean = soundPlayback;

%% resampling probed signal and observation to 28kHz
signals.observ = resample(double(signals.observ), 1, sampleFactor);
signals.clean = resample(double(signals.clean), 1, sampleFactor);


%% Reconstruct RIR from received and observed signals
signals.signalObservFft = fft(signals.observ, setup.signal.nfft); % include direct path comp
% signals.signalObservReflFft = fft(signals.observRefl, setup.signal.nfft);
signals.signalCleanFft = fft(signals.clean, setup.signal.nfft);


% spectrum_analyser(deviceID,inputChannel);
%% Estimator
% PEAK PICKING Single Channel
estimates=peakPeakingPractical(signals,setup);
toaEstimates.peakPicking=estimates.peakPicking_oneCh.toa;

%Single channel EM
estimates=emSingleChan(signals,setup,estimates);
toaEstimates.emOneCh=estimates.emOneCh.toa;

% RNLS UCA
[estimates, costFunction]=rNlsEst(signals,setup);
toaEstimates.rNlsEst=estimates.rNlsEst.toa;

toc

if plotEnable
    figure(200);
    subplot(311)
    plot(rirEst)
    vline(toaEstimates.rNlsEst)
    legend('RIR')
    title('NLS Est')
    subplot(312)
    plot(rirEst)
    vline(toaEstimates.peakPicking)
    legend('RIR')
    title('Peak Picking')
    subplot(313)
    plot(rirEst)
    vline(toaEstimates.emOneCh)
    title('One Channel EM')
    legend('RIR')
end
%% Est. Distance based on TOA
disp('TOA PP, TOA NLS');
% toaEstimates.peakPicking
% peakDist = toaEstimates.peakPicking/setup.signal.sampFreq*setup.room.soundSpeed/2
toas = toaEstimates.rNlsEst
toasEm = toaEstimates.emOneCh
nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2
emDist = toaEstimates.emOneCh/setup.signal.sampFreq*setup.room.soundSpeed/2
if comportEnabled

[serialInterface, serialInterface2]=initializeSerialInterface();
fclose(serialInterface2);
fopen(serialInterface2);
lidarDist = str2num(fscanf(serialInterface2))/100

while isempty(lidarDist)
%             fprintf(serialInterface1,'0')
     lidarDist = str2num(fscanf(serialInterface2))/100;
end
end
%% ROBOT control
if comportEnabled

    if nlsDist < 1
       fprintf(serialInterface, '2');
       pause(1);
       fprintf(serialInterface, '0');
       fclose('all');
    %     if ~isempty(instrfind)
    %         fclose(instrfind);
    %         delete(instrfind);
    %     end
    else
        fprintf(serialInterface, '2');
        pause(1)
        fprintf(serialInterface, '0');

        fclose('all');
    %     if ~isempty(instrfind)
    %         fclose(instrfind);
    %         delete(instrfind);
    %     end

    end
end
pause(2)
end
    if saveData
        save([directoryLoc,'_','robotdata_Office',datestr(now,30)],'recordData', 'soundPlayback', 'rirEst', 'pSNR', 'toas');
        pause(5);
    end
% pSNRObserve = var(recordData(:,1))/var(ambientNoise(:,1));
% snrdBObserve = 10*log10(pSNRObserve)
% end
% spectrum_analyser(deviceID,inputChannel);

% comPort = 'COM5';
% baudRate = 115200;
% serialInterface = serial(comPort, 'BaudRate', baudRate);
% fopen(serialInterface);
% fprintf(serialInterface, '6');
% pause(1);
% fprintf(serialInterface, '0');
