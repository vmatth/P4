%% exponentialSineSweepId
% Estimate the impulse response of a linear system fed with an exponential
% sinusoidal sweep.
%
%% Syntax:
%# estimatedImpulseResponse = ...
%      exponentialSineSweepId(sweptSinusoidalSourceSignal,recordedSignal,...
%      startFreq,stopFreq,samplingFreq,sweepTime,sweepAmp,silenceTime)
%
%% Description:
% Estimate the impulse response of a linear system fed with an exponential
% sinusoidal sweep. The procedure is inspired from
%
% Farina, Angelo. "Advancements in impulse response measurements by sine sweeps."
% Audio Engineering Society Convention 122. Audio Engineering Society, 2007.
%
% Novak, Antonin, et al. "Nonlinear system identification using exponential
% swept-sine signal." Instrumentation and Measurement, IEEE Transactions on
% 59.8 (2010): 2220-2229.
%
% As the source signal, a recorded loop back signal or the ideal swept
% sinusoidal signal can be used. The dynamic range of the amplitude
% response of the swept source signal is limited to 100 dB to avoid numerical
% problems.
%
% * sweptSinusoidalSourceSignal: the source signal (either the ideal swept
% sinusoidal signal or a recorded loop back signal).
% * recordedSignal: the recorded signal.
% * startFreq: the start frequency in Hz of the sweep
% * stopFreq: the stop frequency in Hz of the sweep
% * samplingFreq: the sampling frequency in Hz
% * sweepTime: the sweep time in seconds
% * sweepAmp: the amplitude of the sweep
% * silenceTime: the quiet time (in seconds) added to the end of the sweep
% * estimatedImpulseResponse: the estimated impulse response
%
%% Examples:
% samplingFreq = 48000;
% startFreq = 10;
% stopFreq = samplingFreq/2;
% sweepTime = 10;
% fadeOutTime = 0.01;
% sweepAmp = 1;
% silenceTime = 2;
% sweptSinusoidalSourceSignal = sweepAmp*sweptSinusoid(startFreq,stopFreq,...
%     samplingFreq,sweepTime,fadeOutTime);
% quietTail = zeros(silenceTime*samplingFreq,1);
% sweptSinusoidalSourceSignal = [sweptSinusoidalSourceSignal;quietTail];
% estimatedImpulseResponse = ...
%     exponentialSineSweepId(sweptSinusoidalSourceSignal,recordedSignal,...
%     startFreq,stopFreq,samplingFreq,sweepTime,sweepAmp,silenceTime);
%
%% See also:
% sweptSinusoid
%
function estimatedImpulseResponse = ...
        exponentialSineSweepId(sweptSinusoidalSourceSignal,recordedSignal,...
        startFreq,stopFreq,samplingFreq,sweepTime,sweepAmp,silenceTime)
    % Compute some parameters
    nSourceSignalSamples = length(sweptSinusoidalSourceSignal);
    nRecordedSignalSamples = length(recordedSignal);
    nDft = nSourceSignalSamples+nRecordedSignalSamples-1;
    exponentialFrequencyIncreaseRate = sweepTime/log(stopFreq/startFreq);
    % transform signals into column vectors
    sweptSinusoidalSourceSignal = sweptSinusoidalSourceSignal(:);
    recordedSignal = recordedSignal(:);
    % compute the asymptotic inverse filter with normalisation
    inverseFilter = flipud(sweptSinusoidalSourceSignal).*...
        exp(-(0:nSourceSignalSamples-1)'/(samplingFreq*exponentialFrequencyIncreaseRate))*...
        exp(silenceTime/exponentialFrequencyIncreaseRate)*4*stopFreq/...
        (sweepAmp^2*exponentialFrequencyIncreaseRate*samplingFreq^2);
    % compute the zero-padded DFTs of the signals
    dftInverseFilter = fft(inverseFilter,nDft)*sqrt(nDft);
    dftRecordedSignal = fft(recordedSignal,nDft)/sqrt(nDft);
    % Compute the impulse response
    tmpImpulseResponse = ...
        real(ifft(dftRecordedSignal.*dftInverseFilter));
    % return only the last part of the impulse response
    estimatedImpulseResponse = ...
        tmpImpulseResponse(end-nRecordedSignalSamples+1:end);
end
