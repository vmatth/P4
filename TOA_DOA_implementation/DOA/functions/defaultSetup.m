%% SETUP
function [setup] = defaultSetup(setup,k,cm)
    setup.misc.trueToaDim=2;
    setup.misc.trueToaOrder=1;

    setup.signal.lengthBurst=1500; %Længden af den signal vi sender ud
    setup.signal.lengthSignal=10000; %Længden af hele signalet
    setup.signal.snr=20;
    setup.signal.sdnr=80;
    % setup.signal.sampFreq=44100/2;
    setup.signal.sampFreq=44100;
   
    setup.signal.diffNoiseStr='Turtlebot2_motorSound.wav'; %Baggrundstøj wav fil

    setup.array.micOffset=k;
    setup.array.micNumber=3;
    setup.array.micSpacing=360/setup.array.micNumber;
    setup.array.micAngles=setup.array.micOffset...
        +setup.array.micSpacing*(0:(setup.array.micNumber-1))';
    setup.array.micRadius=0.1;
    setup.array.micPos=setup.array.micRadius...
        *[cosd(setup.array.micAngles),sind(setup.array.micAngles)];

    setup.rirGen.length=[];
    setup.rirGen.micType='omnidirectional';
    setup.rirGen.order=-1;

    setup.room.dimensions=[6,4,3];
    setup.room.T60=0.6;
    setup.room.soundSpeed=343;
    setup.room.distSourceToReceiv=0;
    Dist=setup.room.distSourceToReceiv/setup.room.soundSpeed...
        *setup.signal.sampFreq;
    setup.room.distToWall=1;
    
    %%%%%%%
    lenghtFromMicToSpeaker = 0.1;
    a = lenghtFromMicToSpeaker * sin(deg2rad(45));
    b = a;
    theta = deg2rad(0);
    setup.room.sourcePos=[3,      cm,   1]; % #1
    %setup.room.sourcePos=[5.55,      2,   1]; % #2
    %setup.room.sourcePos=[3,      3.55,   1]; % #3
    %setup.room.sourcePos=[0.45,      2,   1]; % #4
   % setup.room.sourcePos=[xxx,      yyy(j),   1];
    microphone1 = [a*cos(theta)-b*sin(theta)+setup.room.sourcePos(1), a*sin(theta)+b*cos(theta)+setup.room.sourcePos(2)];
    microphone2 = [a*cos(theta)+b*sin(theta)+setup.room.sourcePos(1), a*sin(theta)-b*cos(theta)+setup.room.sourcePos(2)];
    
%    setup.room.receivPos=[microphone1(1), microphone1(2), 1;
%                          microphone2(1), microphone2(2), 1];
    %IMPORTANT: Når positionen ændres skal vi være sikker på at distance
    %intervallet 
    
    %setup.room.sourcePos=[0+setup.room.distToWall,1.3,2.5];
    
     for kk=1:setup.array.micNumber
         setup.room.receivPos(kk,:)=[...
             setup.room.sourcePos(1:2)+setup.array.micPos(kk,:),...
             setup.room.sourcePos(3)-setup.room.distSourceToReceiv];
     end
    

%     setup.EM.plotIterations=1;
%     % setup.EM.minimumDistance=setup.array.micRadius+0.1;
%     setup.EM.minimumDistance=0.5;
%     setup.EM.maximumDistance=2;
%     setup.EM.nRefl=3;
%     setup.EM.nIter=20;
%     setup.EM.beta=1/setup.EM.nRefl;
%     setup.EM.directPathFlag=0;
% 
%     setup.EM.noiseWeighting=1;
end
 
%%Changing variables in setup
%setup.signal.snr=[40 40 40 -40];