function [point] = reflectorPointEstimator(setup, foundArray, distArray, vinkel)

found01 = foundArray(1);
found02 = foundArray(2);
found03 = foundArray(3);
found04 = foundArray(4);

dist1 = distArray(1);
dist2 = distArray(2);
dist3 = distArray(3);
dist4 = distArray(4);

micToSpeakerDistance1 = pdist([setup.room.sourcePos;setup.room.receivPos(1,:)], 'euclidean');

% Hvis man antager at man kender robottens placering i det globale
% cordinatsystem, så kender man også vinkel robotten er drejet med, derfor
% ved man også microfonernes placering. 
vectorLoudspeaker = [setup.room.sourcePos(1),setup.room.sourcePos(2),setup.room.sourcePos(3)];
mic1 = [setup.room.receivPos(1,1); setup.room.receivPos(1,2); setup.room.receivPos(1,3)];
mic2 = [setup.room.receivPos(2,1); setup.room.receivPos(2,2); setup.room.receivPos(2,3)];
mic3 = [setup.room.receivPos(3,1); setup.room.receivPos(3,2); setup.room.receivPos(3,3)];
mic4 = [setup.room.receivPos(4,1); setup.room.receivPos(4,2); setup.room.receivPos(4,3)];
vectormic1 = mic1' - vectorLoudspeaker;
vectormic2 = mic2' - vectorLoudspeaker;
vectormic3 = mic3' - vectorLoudspeaker;
vectormic4 = mic4' - vectorLoudspeaker;

if found01 == 1 
     if abs(vectormic1(1)) > abs(vectormic1(2)) 
        [xmic, ymic] = swap(vectormic1(1),vectormic1(2));
     else
        xmic = vectormic1(1);
        ymic = vectormic1(2);
     end
    distancereal = dist1;
    xmicplot = vectormic1(1);
    ymicplot = vectormic1(2);
elseif found02 == 1
    if abs(vectormic2(1)) > abs(vectormic2(2)) 
        [xmic, ymic] = swap(vectormic2(1),vectormic2(2));
    else
        xmic = vectormic2(1);
        ymic = vectormic2(2);
    end
    distancereal = dist2;
    xmicplot = vectormic2(1);
    ymicplot = vectormic2(2);
elseif found03 == 1
    if abs(vectormic3(1)) > abs(vectormic3(2)) 
        [xmic, ymic] = swap(vectormic3(1),vectormic3(2));
    else
        xmic = vectormic3(1);
        ymic = vectormic3(2);
    end
    distancereal = dist3;
    xmicplot = vectormic3(1);
    ymicplot = vectormic3(2);
elseif found04 == 1
    if abs(vectormic4(1)) > abs(vectormic4(2)) 
        [xmic, ymic] = swap(vectormic4(1),vectormic4(2));
    else
        xmic = vectormic4(1);
        ymic = vectormic4(2);
    end
    distancereal = dist4;
    xmicplot = vectormic4(1);
    ymicplot = vectormic4(2);
end

AfstandWall2Loudspeaker = ((distancereal-abs(ymic))/(2))+abs(ymic); 
vectorLoudspeakerToMic1Real = [xmicplot; ymicplot; 0];
UnitvectorLoudspeakerToMic1 = vectorLoudspeakerToMic1Real / micToSpeakerDistance1;

backRotation = rotz(vinkel);

Realvector = backRotation * UnitvectorLoudspeakerToMic1;
vectorLoudspeakerToMic1 = Realvector * AfstandWall2Loudspeaker;
point = [vectorLoudspeakerToMic1(1)+vectorLoudspeaker(1); vectorLoudspeakerToMic1(2)+vectorLoudspeaker(2); vectorLoudspeakerToMic1(3)+vectorLoudspeaker(3)];
end

