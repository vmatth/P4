 %%

clc
clear
close all
%Stier til mapper hvor der ligger andre mat filer
addpath(genpath('rir')); %Sti til RIR-generatoren
addpath(genpath('lib')); %Alle filer i lib mappen er lavet af jesper & usama
addpath(genpath('functions')); %Alle filer i functions mappen er lavet af os
addpath(genpath('sound')); %Lydfiler som kan bruges. 'continuous' er støj indtil videre


%% Setup
testgrader = linspace(0,360,121); %(0,360,121) hvis 360gradCheck
cm= [0.45,0.55,0.65,0.75];
for jj= 1:1
    for j= 3:3%1:121 hvis 360gradCheck
        for iii= 1:1
    close all
 
% generates a random angle every time the code runs
   % k =testgrader(j)+(10*rand(1,1));
    k = testgrader(j);
     
%     xxx = 0.2 + (0.7)*rand(1,1); 
%     yyy = linspace(0.25,6-0.25,14);
    
%Setup til rum dimensioner, signal variabler osv..
setup=defaultMiscSetup([]); 
setup=defaultSetup(setup,k,cm(jj));


%% Plot Room
%plotRoom(setup); %Plot af rum baseret på setup værdierne

%% Changing variables in setup
%setup.signal.snr=[40 40 40 -40]; (Ved ikke om er vigtig)

%% Rirs
rirs=generateRirs(setup); %Genererer et room impulse response (Kommer fra RIR-generator)
%plotRIRs(rirs); %Plot af room impulse responsen (Jespers funktion)

%% Generate Signals
%White Gaussian
%playback = [randn(setup.signal.lengthBurst,1);... %Første del genererer et random signal
        % zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];% %Anden del tilføjer 0 indtil slutd
playback = [wgn(setup.signal.lengthBurst,1,0);... %Første del genererer et random signal
        zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)]; % Genererer et White Gaussian Noise signal
 %playback = loadWav('Turtlebot2_motorSound.wav', setup, 1); %Loader en fil som bruges til playback... Random=0->tager første 1500 samples, Random=1->tager 1500 sammenhængende samples tilfældigt


 %IMPORTANT: Playback skal være lengthSignalx1 

signals = generateSignals(rirs, [], setup, playback); %Genererer den lyd som mikrofonen hører (udfra RIR) (Jespers funktion)
%plotSignals(signals); %Plot af signalerne 1: Playback. 2: Echo. 3: Støj. 4: Echo + Støj
%crosscorrModeling(signals); %plotter cross-correlation(brugt til test) 

%% PLAY SIGNALS
%soundsc(signals.clean,setup.signal.sampFreq); %Lydene afspilles så i kan høre det :]
%%
%soundsc(signals.reflec(:,1),setup.signal.sampFreq);
%%
%soundsc(signals.noise(:,1)+signals.diffNoise(:,1),setup.signal.sampFreq);
%%
%soundsc(signals.observRefl(:,1),setup.signal.sampFreq);

%% Extract Clean and Observed
[clean, observed] = ExtractCleanAndObserved(signals); %Vi extracter de signaler vi har brug for (Clean = Playback lyden, Observed = Den lyd som mikrofonen hører)

%% Distance calc
[dist1, tau1, MaxIndex1] = DistanceCalc(clean, observed(:,1,1), setup); 
[dist2, tau2, MaxIndex2] = DistanceCalc(clean, observed(:,2,1), setup); 
[dist3, tau3, MaxIndex3] = DistanceCalc(clean, observed(:,3,1), setup); 


% Used in calculating the angle and reflector point
    dist1 = dist1*0.01;
    dist2 = dist2*0.01;
    dist3 = dist3*0.01;
    distArray = [dist1; dist2; dist3];
    
   
    
% function for calculating the angle beteen the robot and the reflector
vectorglobal = UsamaDOA(setup,distArray,testgrader(j),iii);
% Plotting the room and a red circle where the estimator thinks a wall is
% placed


 %  plotRoom(setup,vectorglobal,k);
    


%fileID = fopen('360gradCheck.csv', 'a');
%fprintf(fileID, 'input vinkel: ;%4.2f; maalt vinkel: ;%4.2f; \n',u,vinkeligrader);
%fclose(fileID);
        end
    end
end