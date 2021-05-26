function [forskel] = RMSEafstand(Observed, Real)

d1 = Observed(:,1);
d2 = Observed(:,2);
d3 = Observed(:,3);

r1 = Real(1,1);
r2 = Real(1,2);
r3 = Real(1,3);

d1_median = median(d1);
d2_median = median(d2);
d3_median = median(d3);

forskel1 = r1 - d1_median;
forskel2 = r2 - d2_median;
forskel3 = r3 - d3_median;

forskel = [forskel1; forskel2; forskel3];

end

