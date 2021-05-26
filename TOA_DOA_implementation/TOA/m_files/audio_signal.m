% Generate signals used for the experiment

% Length of the sound burst
setup.signal.lengthBurst = 2000;
% length of the signal
setup.signal.lengthSignal = 20000;
% sampling frequency
sampFreq = 44100;

% Generating a random noise signal followed by a zero padding
signals_clean=[randn(setup.signal.lengthBurst,1);...
        zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];


% Normalize the clean signal so that we don't have clipping when saving
signals.clean = signals_clean/max(abs(signals_clean));

% playback of the signal
soundsc(signals_clean)

% plot of the signal
subplot(211)
plot(signals_clean)

% save signal as a wav file with a stated sampling frequency(optional)
% save('gaussian_white_noise.mat','signals_clean','sampFreq')

% check to see if the audio file is the same as the generated signal
load('gaussian_white_noise.mat');
subplot(212)
plot(signals_clean);