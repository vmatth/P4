function [RMSE] = RMSEphysicalSetup(Observed, Real)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
d1 = Observed(:,1);
d2 = Observed(:,2);
d3 = Observed(:,3);

r1 = Real(1,1);
r2 = Real(1,2);
r3 = Real(1,3);


    RMSE_dist1 = sqrt(mean((r1 - d1).^2));  % Root Mean Squared Error
    RMSE_dist2 = sqrt(mean((r2 - d2).^2));  % Root Mean Squared Error
    RMSE_dist3 = sqrt(mean((r3 - d3).^2));  % Root Mean Squared Error
    
    RMSE = [RMSE_dist1; RMSE_dist2; RMSE_dist3];

end

