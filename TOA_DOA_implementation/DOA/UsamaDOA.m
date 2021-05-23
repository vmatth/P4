function [reflectorPoint] = UsamaDOA(setup,distArray,grader,iii)

Sx=setup.room.sourcePos(1,1);
Sy=setup.room.sourcePos(1,2);

x1=setup.room.receivPos(1,1);
y1=setup.room.receivPos(1,2);
d1=distArray(1);

[xmic1,ymic1,radii1] = circleCalculator(x1,y1,d1);
plot(xmic1,ymic1,'c')
axis equal
hold on


x2=setup.room.receivPos(2,1);
y2=setup.room.receivPos(2,2);
d2=distArray(2);

[xmic2,ymic2,radii2] = circleCalculator(x2,y2,d2);
plot(xmic2,ymic2,'m')
axis equal
hold on


x3=setup.room.receivPos(3,1);
y3=setup.room.receivPos(3,2);
d3=distArray(3);

[xmic3,ymic3,radii3] = circleCalculator(x3,y3,d3);
plot(xmic3,ymic3,'g')
axis equal
hold on


plot(Sx,Sy,'r*')
hold on
wall = [1.5,0;
         6,0];
plot(wall(:,1),wall(:,2),'b')
hold on

% formel 25
A = [2*(x1-x3), 2*(y1-y3);
     2*(x2-x3), 2*(y2-y3)];
 
 % formel 26
b = [(x1*x1)-(x3*x3) + (y1*y1)-(y3*y3) + (d3*d3)-(d1*d1);
     (x2*x2)-(x3*x3) + (y2*y2)-(y3*y3) + (d3*d3)-(d2*d2)];
 
 % formel 24
 r= ((transpose(A)*A)\transpose(A)*b);
 
vectorSr = [r(1,1)-Sx;
           r(2,1)-Sy];


            %plot(vectorSr(1,1),vectorSr(2,1),'r+')
plot(r(1,1),r(2,1),'r+')

lengthSr = sqrt((vectorSr(1,1)*vectorSr(1,1))+(vectorSr(2,1)*vectorSr(2,1)));

newVectorLength = lengthSr / 2;

unitVector = vectorSr/lengthSr;

reflectorPoint = unitVector * newVectorLength;
reflectorPoint(1) = reflectorPoint(1) + setup.room.sourcePos(1,1);
reflectorPoint(2) = reflectorPoint(2) + setup.room.sourcePos(1,2);
plot(reflectorPoint(1,1),reflectorPoint(2,1),'rx')

plot(x1,y1,'rs')
hold on
plot(x2,y2,'rd')
hold on
plot(x3,y3,'rp')
hold on
%ylim([-1,2])
legend('d_1', 'd_2', 'd_3', 'Loudspeaker', 'Echo reflector','r_s', 'r_p', 'Microphone 1','Microphone 2','Microphone 3')

%  savestr = sprintf('%4.2f_DOA_at_%4.2f_angle.png',grader,iii);
%      saveas(gcf,savestr);


difX = reflectorPoint(1,1) - Sx;
difY = reflectorPoint(2,1) - 0;

vectorWall = [Sx; 0];

vectorLoudSpeakerTowall = [vectorWall(1)-Sx;
                           vectorWall(2)-Sy];
                       
vectorReflector = [vectorLoudSpeakerTowall(1)+difX;
                   vectorLoudSpeakerTowall(2)+difY];
               
lengthvectorReflector = sqrt(vectorReflector(1)*vectorReflector(1)+vectorReflector(2)*vectorReflector(2));
lengthvectorLoudSpeakerTowall = sqrt(vectorLoudSpeakerTowall(1)*vectorLoudSpeakerTowall(1)+vectorLoudSpeakerTowall(2)*vectorLoudSpeakerTowall(2));


vinkel = acos((dot(vectorLoudSpeakerTowall,vectorReflector))/(lengthvectorReflector*lengthvectorLoudSpeakerTowall));
vinkelGrader = rad2deg(vinkel);          
filename = sprintf('test_req9_at_multiple_angles_and_distances2_19_05_2021.csv');
fileID = fopen(filename, 'a');
fprintf(fileID, 'Distance: ;%f; Angle: ;%4.2f; Test number: ;%4.2f; Difference x: ;%f; Difference y: ;%f; AngleDiff: ;%f; AngleDiffdegree: ;%4.2f; \n',setup.room.sourcePos(2), grader, iii, difX,difY, vinkel, vinkelGrader);
fclose(fileID);

end
