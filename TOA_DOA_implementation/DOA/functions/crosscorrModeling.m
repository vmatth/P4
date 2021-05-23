function crosscorrModeling(signals)
x=signals.clean(:,1);
y=signals.observRefl(:,1);
figure(111)
%[c,lags] = xcorr(y,x);
%stem(lags,c)
crosscorr(x,y,'NumLags',500);
 xlabel('Time [samples]');
 title('Cross-correlation');
end

