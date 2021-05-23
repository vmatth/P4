function [xCircle, yCircle, R] = circleCalculator(xR,yR,R)

%% Circles
alpha = linspace(0,2*pi,360)'; % angle to calculate full circle, 0 to 2pi

xCircle=R * cos(alpha)+xR; % x coordinates of the points that form the circles
yCircle=R * sin(alpha)+yR; % y coordinates of the poitns that form the circles
end