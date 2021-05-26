function [point] = reflectorPointEstimator3v2(setup, foundArray, distArray, vinkel)

found01 = foundArray(1);
found02 = foundArray(2);
found03 = foundArray(3);


dist1 = distArray(1);
dist2 = distArray(2);
dist3 = distArray(3);


micToSpeakerDistance1 = pdist([setup.room.sourcePos;setup.room.receivPos(1,:)], 'euclidean');

% Hvis man antager at man kender robottens placering i det globale
% koordinatsystem, så kender man også vinkel robotten er drejet med, derfor
% ved man også microfonernes placering. 
vectorLoudspeaker = [setup.room.sourcePos(1),setup.room.sourcePos(2),setup.room.sourcePos(3)];
mic1 = [setup.room.receivPos(1,1); setup.room.receivPos(1,2); setup.room.receivPos(1,3)];
mic2 = [setup.room.receivPos(2,1); setup.room.receivPos(2,2); setup.room.receivPos(2,3)];
mic3 = [setup.room.receivPos(3,1); setup.room.receivPos(3,2); setup.room.receivPos(3,3)];
vectormic1 = mic1' - vectorLoudspeaker;
vectormic2 = mic2' - vectorLoudspeaker;
vectormic3 = mic3' - vectorLoudspeaker;


if found01 == 1 
        xmic = vectormic1(1);
        ymic = vectormic1(2);
    distancereal = dist1;
elseif found02 == 1
        xmic = vectormic2(1);
        ymic = vectormic2(2);
    distancereal = dist2;
elseif found03 == 1
        xmic = vectormic3(1);
        ymic = vectormic3(2);
    distancereal = dist3;
end

AfstandWall2Loudspeaker = ((distancereal-abs(ymic))/(2))+abs(ymic); 
vectorLoudspeakerToMic1Real = [vectormic1(1); vectormic1(2); 0];

UnitvectorLoudspeakerToMic1 = vectorLoudspeakerToMic1Real / micToSpeakerDistance1;

if dist3<dist2 && dist1<dist2 && vectormic1(1)>0 && vectormic1(2)<0 && vectormic3(1)<0 && vectormic3(2)<0
    backRotation = rotz(-vinkel);
else
    backRotation = rotz(vinkel);
end
%backRotation = rotz(vinkel);

Realvector = backRotation * UnitvectorLoudspeakerToMic1;


vectorLoudspeakerToWall = Realvector * AfstandWall2Loudspeaker;

point = [vectorLoudspeakerToWall(1)+vectorLoudspeaker(1); vectorLoudspeakerToWall(2)+vectorLoudspeaker(2); vectorLoudspeakerToWall(3)+vectorLoudspeaker(3)];
end

