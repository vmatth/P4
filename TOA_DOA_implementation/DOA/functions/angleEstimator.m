function [vinkel, Realvinkel] = angleEstimator(setup,distArray, foundArray, sortedVals)


dist1 = distArray(1);
dist2 = distArray(2);
dist3 = distArray(3);
dist4 = distArray(4);

found01 = foundArray(1);
found02 = foundArray(2);
found03 = foundArray(3);
found04 = foundArray(4);


distmicmic = sqrt((setup.room.receivPos(2,1)-setup.room.receivPos(1,1))^2 ...
                   +(setup.room.receivPos(2,2)-setup.room.receivPos(1,2))^2); 


if (found01(1)) == 1 % then mic1 is closest to the wall
    text = 'inde i mic1 if statement';
    found1 = find(sortedVals==dist2);
    found2 = find(sortedVals==dist4);
    if (found1(1)) == 2 % mic2 is second closest to the wall
        deltaDist = dist2-dist1;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = pi/4 - vinkel;
        text = 'mic1 og mic2';
        kvadrant = 11;
    elseif (found2(1)) == 2 % mic4 is second closest to the wall
        deltaDist = dist4-dist1;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = 2*pi - (pi/4 - vinkel);
        text = 'mic1 og mic4';
        kvadrant = 14;
    elseif dist1 == dist4
        vinkel = deg2rad(45);
        Realvinkel = deg2rad(315);
        text = 'mic1 og mic4 snyd';
        kvadrant = 141;
    elseif dist1 == dist2
        vinkel = deg2rad(45);
        Realvinkel = deg2rad(45);
        text = 'mic1 og mic2 snyd';
        kvadrant = 142;
    end
  
elseif  (found02(1)) == 1 % then mic2 is closest to the wall
    text = 'inde i mic2 if statement';
    found1 = find(sortedVals==dist1);
    found2 = find(sortedVals==dist3);
    if (found1(1)) == 2 % mic1 is second closest to the wall
        deltaDist = dist1-dist2;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = pi/4 + vinkel; 
        text = 'mic2 og mic1';
        kvadrant = 21;
    elseif (found2(1)) == 2 % mic3 is second closest to the wall
        deltaDist = dist3-dist2;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = pi/2 + (pi/4-vinkel);
        text = 'mic2 og mic3';
        kvadrant = 22;
     elseif dist2 == dist3
         vinkel = deg2rad(45);
         Realvinkel = deg2rad(135);
         text = 'mic2 og mic3 snyd';
         kvadrant = 221;
    end

elseif  (found03(1)) == 1 % then mic3 is closest to the wall
    text = 'inde i mic3 if statement';
    found1 = find(sortedVals==dist2);
    found2 = find(sortedVals==dist4);
    if (found1(1)) == 2 % mic2 is second closest to the wall
        deltaDist = dist2-dist3;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = pi- (pi/4 - vinkel);
        text = 'mic3 og mic2';
        kvadrant = 32;
    elseif (found2(1)) == 2  % mic4 is second closest to the wall
        deltaDist = dist4-dist3;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = pi + (pi/4 -vinkel);
        text = 'mic3 og mic4';
        kvadrant = 33;
     elseif dist4 == dist3
         vinkel= deg2rad(45);
         Realvinkel = deg2rad(225);
         text = 'mic3 og mic4 snyd';
         kvadrant = 331;
    end
   
elseif  (found04(1)) == 1 % then mic4 is closest to the wall
    text = 'inde i mic4 if statement';
    found1 = find(sortedVals==dist3);
    found2 = find(sortedVals==dist1);
    if (found1(1)) == 2 % mic3 is second closest to the wall
        deltaDist = dist3-dist4;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = (3*pi)/2 - (pi/4 -vinkel);
        text = 'mic4 og mic3';
        kvadrant = 43;
    elseif (found2(1)) == 2 % mic1 is second closest to the wall
        deltaDist = dist1-dist4;
        vinkel = asin(deltaDist/distmicmic);
        Realvinkel = (3*pi)/2 + (pi/4 -vinkel);
        text = 'mic4 og mic1';
        kvadrant = 44;
    end
end
end

