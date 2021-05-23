function [vinkel, Realvinkel, text] = angleEstimator3(setup,distArray, foundArray, sortedVals)


dist1 = distArray(1);
dist2 = distArray(2);
dist3 = distArray(3);

found01 = foundArray(1);
found02 = foundArray(2);
found03 = foundArray(3);



distmicmic = sqrt((setup.room.receivPos(2,1)-setup.room.receivPos(1,1))^2 ...
                   +(setup.room.receivPos(2,2)-setup.room.receivPos(1,2))^2); 


if (found01(1)) == 1 % then mic1 is closest to the wall
    text = 'inde i mic1 if statement';
    found1 = find(sortedVals==dist2);
    found2 = find(sortedVals==dist3);
    if (found1(1)) == 2 % mic2 is second closest to the wall
        deltaDist = dist2-dist1;
        vinkel = deg2rad(360) - (deg2rad(60) - asin(deltaDist/distmicmic));
        Realvinkel = 0;
        text = 'mic1 og mic2';
        kvadrant = 11;
    elseif (found2(1)) == 2 % mic3 is second closest to the wall
        deltaDist = dist3-dist1;
        vinkel = deg2rad(60) - asin(deltaDist/distmicmic);
        Realvinkel = 0;
        text = 'mic1 og mic3';
        kvadrant = 14;
        
    elseif (found02(1) == 1)
        vinkel = deg2rad(300);
        Realvinkel = 0;
        text = 'mic1 og mic2';
            
    elseif (found03(1) == 1)
        vinkel = deg2rad(60);
        Realvinkel = 0;
        text = 'mic1 og mic3';
    end
  
elseif  (found02(1)) == 1 % then mic2 is closest to the wall
    text = 'inde i mic2 if statement';
    found1 = find(sortedVals==dist1);
    found2 = find(sortedVals==dist3);
    if (found1(1)) == 2 % mic1 is second closest to the wall
        deltaDist = dist1-dist2;
        vinkel = deg2rad(300)-asin(deltaDist/distmicmic); %Måske fucker den her op, burde ikke tho. (360-(pi/3 + asin(deltaDist/distmicmic)))
        Realvinkel = 0;
        text = 'mic2 og mic1';
        kvadrant = 21;
    elseif (found2(1)) == 2
        deltaDist = dist1-dist3;
        vinkel = deg2rad(60) - asin(deltaDist/distmicmic)+deg2rad(180);
        Realvinkel = 0;  
            
    elseif (found03(1) == 1)
        vinkel = deg2rad(180);
        Realvinkel = 0;
        text = 'mic2 og mic3';
    end

elseif  (found03(1)) == 1 % then mic3 is closest to the wall
    text = 'inde i mic3 if statement';
    found1 = find(sortedVals==dist1);
    found2 = find(sortedVals==dist2);
    if (found1(1)) == 2 % mic1 is second closest to the wall
        deltaDist = dist1-dist3;
        vinkel = deg2rad(60) + asin(deltaDist/distmicmic);
        Realvinkel = 0;
        text = 'mic3 og mic1';
        kvadrant = 32;
     elseif (found2(1)) == 2
        deltaDist = dist2-dist3;
        vinkel = deg2rad(60) - asin(deltaDist/distmicmic)+deg2rad(120);
        Realvinkel = 0;
    end
   
end
end

