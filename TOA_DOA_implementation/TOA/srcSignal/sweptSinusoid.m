%% sweptSinusoid
% Produces either a linearly and exponentially swept sinusoidal signal.
%
%% Syntax: 
%# sweptSinusoidalSignal = sweptSinusoid(startFreq,stopFreq,samplingFreq,...
%                      sweepTime,fadeOutTime,sweepIsExponential)
%
%% Description:
% Produces a swept sinusoidal signal as recommended in
%
% Farina, Angelo. "Advancements in impulse response measurements by sine sweeps."
% Audio Engineering Society Convention 122. Audio Engineering Society, 2007.
%
% To avoid a broadband click in the end of the sweep, a fade out of length
% specified by the fadeOutTime parameter can be added. The fade out window is
% the last half of a Blackman window.
%
% * startFreq: start frequency of the sweep in Hz
% * stopFreq: stop frequency of the sweep in Hz
% * samplingFreq: sampling frequency in Hz
% * sweepTime: sweep time in seconds
% * fadeOutTime: (optional) fade out time in seconds. The default is 10 ms
% * sweepIsExponential: (optional) Flag for specifying whether the sweep is
%   exponential or linear. The default is true (exponential sweep).
% * sweptSinusoidalSignal: the swept sinusoidal signal
%
%% Examples:
% samplingFreq = 48000;
% startFreq = 10;
% stopFreq = samplingFreq/2;
% sweepTime = 10;
% fadeOutTime = 0.01;
% sweptSinusoidalSourceSignal = sweptSinusoid(startFreq,stopFreq,samplingFreq,...
%     sweepTime,fadeOutTime);
%
%% See also:
% exponentialSineSweepId
%
function sweptSinusoidalSignal = sweptSinusoid(startFreq,stopFreq,samplingFreq,...
        sweepTime,fadeOutTime,sweepIsExponential)
    if nargin < 5
        fadeOutTime = 0.01;
    end
    if nargin < 6
        sweepIsExponential = true;
    end
    % number of data points
    nData = floor(samplingFreq*sweepTime)+1;
    % time vector
    timeVector = (0:nData-1)'/samplingFreq;
    if sweepIsExponential
        % exponential sweep (the modulo operation is only added for numerical
        % reasons)
        tmpParam = sweepTime/log(stopFreq/startFreq);
        sweptSinusoidalSignal = ...
            sin(2*pi*mod(startFreq*tmpParam*(exp(timeVector/tmpParam)-1),1));
    else
        % linear sweep
        k = (stopFreq-startFreq)/sweepTime;
        sweptSinusoidalSignal = sin(2*pi*(startFreq*timeVector+k*timeVector.^2/2));
    end
    % apply triangular fade out
    nFadeOutSamples = ceil(fadeOutTime*samplingFreq);
    window = blackman(2*nFadeOutSamples);
    sweptSinusoidalSignal(nData-nFadeOutSamples+1:nData) = ...
        sweptSinusoidalSignal(nData-nFadeOutSamples+1:nData).*...
        window((1:nFadeOutSamples)+nFadeOutSamples);
end
