
clear all
close all
clc
table = readtable('testingAtDifferentDistances_19-05-2021.csv');

dist1 = table.Var1(:);
dist2 = table.Var2(:);
dist3 = table.Var3(:);


% csv file -> 65cm , 45cm , 55cm , 75cm , 85cm , 95cm , 105cm , 115cm ,
% 125cm , 135cm

%Midt afstand
% 45cm 65cm 85cm 100cm 120cm
%Real1 = [45+27, 55+37, 65+47, 75+57, 85+67, 95+77, 105+87, 115+97, 125+107, 135+117]';
%Real2 = [45+57.5, 55+67.5, 65+77.5,  75+87.5, 85+97.5, 95+107.5, 105+117.5, 115+127.5, 125+137.5, 135+147.5]';
%Real3 = Real2;

Real1 = [25+5, 35+15, 45+25, 55+35, 65+45, 75+45, 85+55, 95+65, 105+85, 115+95, 125+105, 135+115]';%, 145+125, 155+135, 165+145, 175+155, 185+165, 195+175, 205+185, 215+195, 225+205, 235+215, 245+225, 255+235];
Real2 = [25+35, 35+45, 45+55, 55+65, 65+75, 75+85, 85+95, 95+105, 105+115, 115+125, 125+135, 135+145]';%, 145+155, 155+165, 165+175, 175+185, 185+195, 195+205, 205+215, 215+225, 225+235, 235+245, 245+255, 255+265];
Real3 = Real2;
distArray = [dist1, dist2, dist3];
realArray = [Real1, Real2, Real3];


%% 45 cm
%RMSE1_45cm = RMSEphysicalSetup(distArray(101:200,1:3), realArray(1,:));

%% 55 cm
%RMSE2_55cm = RMSEphysicalSetup(distArray(201:300,1:3), realArray(2,:));

%% 65 cm
%RMSE3_65cm = RMSEphysicalSetup(distArray(1:100,1:3), realArray(3,:));

%% 75 cm
%RMSE4_75cm = RMSEphysicalSetup(distArray(301:400,1:3), realArray(4,:));

%% 85 cm
%RMSE5_85cm = RMSEphysicalSetup(distArray(401:500,1:3), realArray(5,:));

%% 95 cm
%RMSE6_95cm = RMSEphysicalSetup(distArray(501:600,1:3), realArray(6,:));

%% 105 cm
%RMSE7_105cm = RMSEphysicalSetup(distArray(601:700,1:3), realArray(7,:));

%% 115 cm
%RMSE8_115cm = RMSEphysicalSetup(distArray(701:800,1:3), realArray(8,:));

%% 125 cm
%RMSE9_125cm = RMSEphysicalSetup(distArray(801:900,1:3), realArray(9,:));

%% 135 cm
%RMSE10_135cm = RMSEphysicalSetup(distArray(901:1000,1:3), realArray(10,:));

%% plot
% xakse = [1,2,3];
% figure('Name','RMSE')
% plot(xakse, RMSE1_45cm,'r')
% hold on
% plot(xakse, RMSE2_55cm)
% hold on
% plot(xakse, RMSE3_65cm)
% hold on
% plot(xakse, RMSE4_75cm)
% hold on
% plot(xakse, RMSE5_85cm)
% hold on
% plot(xakse, RMSE6_95cm)
% hold on
% plot(xakse, RMSE7_105cm)
% hold on
% plot(xakse, RMSE8_115cm)
% hold on
% plot(xakse, RMSE9_125cm)
% hold on
% plot(xakse, RMSE10_135cm)
% 
% set(gca,'XTick',1:1:3)
% set(gca,'XTickLabel',{'Microphone 1','Microphone 2','Microphone 3'})
% ylabel('RMSE [cm]');
% title('Root Mean Square Error');
% legend('45 cm','55 cm','65 cm','75 cm', '85 cm','95 cm','105 cm','115 cm','125 cm','135 cm')
% 

%% 25 cm
forskel25cm = RMSEafstand(distArray(1:100,1:3), realArray(1,:));

%% 35 cm
forskel35cm = RMSEafstand(distArray(101:200,1:3), realArray(2,:));

%% 45 cm
forskel45cm = RMSEafstand(distArray(201:300,1:3), realArray(3,:));

%% 55 cm
forskel55cm = RMSEafstand(distArray(301:400,1:3), realArray(4,:));

%% 65 cm
forskel65cm = RMSEafstand(distArray(401:500,1:3), realArray(5,:));

%% 75 cm
forskel75cm = RMSEafstand(distArray(501:600,1:3), realArray(6,:));

%% 85 cm
forskel85cm = RMSEafstand(distArray(601:700,1:3), realArray(7,:));

%% 95 cm
forskel95cm = RMSEafstand(distArray(701:800,1:3), realArray(8,:));

%% 105 cm
forskel105cm = RMSEafstand(distArray(801:900,1:3), realArray(9,:));

%% 115 cm
forskel115cm = RMSEafstand(distArray(901:1000,1:3), realArray(10,:));

%% 125 cm
forskel125cm = RMSEafstand(distArray(1001:1100,1:3), realArray(11,:));

%% 135 cm
forskel135cm = RMSEafstand(distArray(1101:1200,1:3), realArray(12,:));

%% 145 cm
%forskel145cm = RMSEafstand(distArray(1201:1300,1:3), realArray(13,:));

%% 155 cm
%forskel155cm = RMSEafstand(distArray(1301:1400,1:3), realArray(14,:));

%% 165 cm
%forskel165cm = RMSEafstand(distArray(1401:1500,1:3), realArray(15,:));

%% 175 cm
%forskel175cm = RMSEafstand(distArray(1501:1600,1:3), realArray(16,:));

%% 185 cm
%forskel185cm = RMSEafstand(distArray(1601:1700,1:3), realArray(17,:));

%% 195 cm
%forskel195cm = RMSEafstand(distArray(1701:1800,1:3), realArray(18,:));

%% 205 cm
%forskel205cm = RMSEafstand(distArray(1801:1900,1:3), realArray(19,:));

%% 215 cm
%forskel215cm = RMSEafstand(distArray(1901:2000,1:3), realArray(20,:));

%% 225 cm
%forskel225cm = RMSEafstand(distArray(2001:2100,1:3), realArray(21,:));

%% 235 cm
%forskel235cm = RMSEafstand(distArray(2101:2200,1:3), realArray(22,:));

%% 245 cm
%forskel245cm = RMSEafstand(distArray(2201:2300,1:3), realArray(23,:));

%% 255 cm
%forskel255cm = RMSEafstand(distArray(2301:2400,1:3), realArray(24,:));

%% mic 1
xakse2 = [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35];%, 1.45, 1.55, 1.65, 1.75, 1.85, 1.95, 2.05, 2.15, 2.25, 2.35, 2.45, 2.55];
mic1 = [forskel25cm(1,1), forskel35cm(1,1), forskel45cm(1,1), forskel55cm(1,1), forskel65cm(1,1), forskel75cm(1,1), forskel85cm(1,1),forskel95cm(1,1), forskel105cm(1,1), forskel115cm(1,1), forskel125cm(1,1), forskel135cm(1,1)];%, forskel145cm(1,1), forskel155cm(1,1), forskel165cm(1,1), forskel175cm(1,1), forskel185cm(1,1), forskel195cm(1,1), forskel205cm(1,1), forskel215cm(1,1), forskel225cm(1,1), forskel235cm(1,1), forskel245cm(1,1), forskel255cm(1,1)];
mic2 = [forskel25cm(2,1), forskel35cm(2,1), forskel45cm(2,1), forskel55cm(2,1), forskel65cm(2,1), forskel75cm(2,1), forskel85cm(3,1),forskel95cm(2,1), forskel105cm(2,1), forskel115cm(2,1), forskel125cm(2,1), forskel135cm(2,1)];%, forskel145cm(2,1), forskel155cm(2,1), forskel165cm(2,1), forskel175cm(2,1), forskel185cm(2,1), forskel195cm(2,1), forskel205cm(2,1), forskel215cm(2,1), forskel225cm(2,1), forskel235cm(2,1), forskel245cm(2,1), forskel255cm(2,1)];
mic3 = [forskel25cm(2,1), forskel35cm(2,1), forskel45cm(3,1), forskel55cm(3,1), forskel65cm(3,1), forskel75cm(3,1), forskel85cm(3,1),forskel95cm(3,1), forskel105cm(3,1), forskel115cm(3,1), forskel125cm(3,1), forskel135cm(3,1)];%, forskel145cm(3,1), forskel155cm(3,1), forskel165cm(3,1), forskel175cm(3,1), forskel185cm(3,1), forskel195cm(3,1), forskel205cm(3,1), forskel215cm(3,1), forskel225cm(3,1), forskel235cm(3,1), forskel245cm(3,1), forskel255cm(3,1)];
figure(10);
y123 = [0,0,0,0,0,0,0,0,0,0,0,0];%,0,0,0,0,0,0,0,0,0,0,0,0]; 
x = xakse2;
yneg1 = [25*0.9-25 35*0.9-35 45*0.9-45 55*0.9-55 65*0.9-65 75*0.9-75 85*0.9-85 95*0.9-95 105*0.9-105 115*0.9-115 125*0.9-125 135*0.9-135 ];%145*0.9-145 155*0.9-155 165*0.9-165 175*0.9-175 185*0.9-185 195*0.9-195 205*0.9-205 215*0.9-215 225*0.9-225 235*0.9-235 245*0.9-245 255*0.9-255];
ypos1 = [25*1.1-25 35*1.1-35 45*1.1-45 55*1.1-55 65*1.1-65 75*1.1-75 85*1.1-85 95*1.1-95 105*1.1-105 115*1.1-115 125*1.1-125 135*1.1-135 ];%145*1.1-145 155*1.1-155 165*1.1-165 175*1.1-175 185*1.1-185 195*1.1-195 205*1.1-205 215*1.1-215 225*1.1-225 235*1.1-235 245*1.1-245 255*1.1-255];

plot(xakse2, mic1,'-o')
hold on
errorbar(x,y123,yneg1,ypos1,'r')
ylabel('Median error [cm]'); xlabel('Test distances [m]');
title('Microphone 1 - median error');
legend('Median error distance', '10% accuracy margin', 'Location','northwest')
        
grid on

figure(20);
plot(xakse2, mic2,'-o')
hold on
errorbar(x,y123,yneg1,ypos1,'r')
ylabel('Median error [cm]'); xlabel('Test distances [m]');
title('Microphone 2 - median error');
legend('Median error distance', '10% accuracy margin','Location','northwest')
grid on

figure(30);
plot(xakse2, mic3,'-o')
hold on
errorbar(x,y123,yneg1,ypos1,'r')
ylabel('Median error [cm]'); xlabel('Test distances [m]');
title('Microphone 3 - median error');
legend('Median error distance', '10% accuracy margin', 'Location','northwest')
grid on





%% histogram af hvre afstand
figure(222)
edges = linspace(70, 110, 110-70+1); % Create 20 bins.
histogram(distArray(101:200,1),'BinEdges',edges)
grid on;
xlim([70, 110]);
hold on 
histogram(distArray(101:200,2))
hold on 
histogram(distArray(101:200,3))
ylabel('Bin count'); xlabel('Measured Reflected Distances [cm]');
title('45 cm distance');
legend('Microphone 1 (72cm)','Microphone 2 (102.5cm)','Microphone 3 (102.5cm)')


%% histogram ish
% 25 cm
findvalue(distArray(1:100,1), distArray(1:100,2), distArray(1:100,3),'25 cm distance');
% 35 cm
findvalue(distArray(101:200,1), distArray(101:200,2), distArray(101:200,3),'35 cm distance');
% 45 cm
findvalue(distArray(201:300,1), distArray(201:300,2), distArray(201:300,3),'45 cm distance');
% 55 cm
findvalue(distArray(301:400,1),distArray(301:400,2),distArray(301:400,3),'55 cm distance');
% 65 cm
findvalue(distArray(401:500,1),distArray(401:500,2),distArray(401:500,3),'65 cm distance');
% 75 cm
findvalue(distArray(501:600,1),distArray(501:600,2),distArray(501:600,3),'75 cm distance');
% 85 cm
findvalue(distArray(601:700,1),distArray(601:700,2),distArray(601:700,3),'85 cm distance');
% 95 cm
findvalue(distArray(701:800,1),distArray(701:800,2),distArray(701:800,3),'95 cm distance');
% 105 cm
findvalue(distArray(801:900,1),distArray(801:900,2),distArray(801:900,3),'105 cm distance');
% 115 cm
findvalue(distArray(901:1000,1),distArray(901:1000,2),distArray(901:1000,3),'115 cm distance');
% 125 cm
findvalue(distArray(1001:1100,1),distArray(1001:1100,2),distArray(1001:1100,3),'125 cm distance');
% 135 cm
findvalue(distArray(1101:1200,1),distArray(1101:1200,2),distArray(1101:1200,3),'135 cm distance');
% 145 cm
%findvalue(distArray(1201:1300,1),distArray(1201:1300,2),distArray(1201:1300,3),'145 cm distance');
% 155 cm
%findvalue(distArray(1301:1400,1),distArray(1301:1400,2),distArray(1301:1400,3),'155 cm distance');
% 165 cm
%findvalue(distArray(1401:1500,1),distArray(1401:1500,2),distArray(1401:1500,3),'165 cm distance');
% 175 cm
%findvalue(distArray(1501:1600,1),distArray(1501:1600,2),distArray(1501:1600,3),'175 cm distance');
% 185 cm
%findvalue(distArray(1601:1700,1),distArray(1601:1700,2),distArray(1601:1700,3),'185 cm distance');
% 195 cm
%findvalue(distArray(1701:1800,1),distArray(1701:1800,2),distArray(1701:1800,3),'195 cm distance');
% 205 cm
%findvalue(distArray(1801:1900,1),distArray(1801:1900,2),distArray(1801:1900,3),'205 cm distance');
% 215 cm
%findvalue(distArray(1901:2000,1),distArray(1901:2000,2),distArray(1901:2000,3),'215 cm distance');
% 225 cm
%findvalue(distArray(2001:2100,1),distArray(2001:2100,2),distArray(2001:2100,3),'225 cm distance');
% 235 cm
%findvalue(distArray(2101:2200,1),distArray(2101:2200,2),distArray(2101:2200,3),'235 cm distance');
% 245 cm
%findvalue(distArray(2201:2300,1),distArray(2201:2300,2),distArray(2201:2300,3),'245 cm distance');
% 255 cm
%findvalue(distArray(2301:2400,1),distArray(2301:2400,2),distArray(2301:2400,3),'255 cm distance');


%%
% [mic1x,mic1y]=findvalue(distArray(101:200,1));
% [mic2x,mic2y]=findvalue(distArray(101:200,2));
% [mic3x,mic3y]=findvalue(distArray(101:200,3));
% figure(123)
% stem(mic1y,mic1x,'filled')
% hold on
% stem(mic2y,mic2x,'filled')
% hold on
% stem(mic3y,mic3x,'filled')
% ylabel('Count'); xlabel('Measured Reflected Distances [cm]');
% ylim([0,110])
% title('45 cm distance');
% legend('Microphone 1 (72cm)','Microphone 2 (102.5cm)','Microphone 3 (102.5cm)', 'Location','southwest')
% 

%% 95 cm
% [mic951x,mic951y]=findvalue(distArray(501:600,1));
% [mic952x,mic952y]=findvalue(distArray(501:600,2));
% [mic953x,mic953y]=findvalue(distArray(501:600,3));
% figure(123)
% stem(mic1y,mic951x,'filled')
% hold on
% stem(mic2y,mic952x,'filled')
% hold on
% stem(mic3y,mic953x,'filled')
% ylabel('Count'); xlabel('Measured Reflected Distances [cm]');
% ylim([0,110])
% title('45 cm distance');
% legend('Microphone 1 (72cm)','Microphone 2 (102.5cm)','Microphone 3 (102.5cm)', 'Location','southwest')

%%
% figure(111)
% tiledlayout(10,3) % Requires R2019b or later
% 
% % first row
% nexttile
% histogram(distArray(101:200,1))
% grid on;
% xlim([0, 300]);
% title('Microphone 1 45 cm (72)')
% nexttile
% histogram(distArray(101:200,2), 11)
% title('Microphone 2 45 cm (102.5)')
% nexttile
% histogram(distArray(101:200,3), 11)
% title('Microphone 3 45 cm (102.5)')
% 
% % second row
% nexttile
% histogram(distArray(201:300,1), 11)
% title('Microphone 1 55 cm (92)')
% nexttile
% histogram(distArray(201:300,2), 11)
% title('Microphone 2 55 cm (122.5)')
% nexttile
% histogram(distArray(201:300,3), 11)
% title('Microphone 3 55 cm (122.5)')
% 
% % Thrid row
% nexttile
% histogram(distArray(1:100,1), 11)
% title('Microphone 1 65 cm (112)')
% nexttile
% histogram(distArray(1:100,2), 11)
% title('Microphone 2 65 cm (142.5)')
% nexttile
% histogram(distArray(1:100,3), 11)
% title('Microphone 3 65 cm (142.5)')
% 
% % 4. row
% nexttile
% histogram(distArray(301:400,1), 11)
% title('Microphone 1 75 cm (132)')
% nexttile
% histogram(distArray(301:400,2), 11)
% title('Microphone 2 75 cm (162.5)')
% nexttile
% histogram(distArray(301:400,3), 11)
% title('Microphone 3 75 cm (162.5)')
% 
% % 5. row
% nexttile
% histogram(distArray(401:500,1), 11)
% title('Microphone 1 85 cm (152)')
% nexttile
% histogram(distArray(401:500,2), 11)
% title('Microphone 2 85 cm (182.5)')
% nexttile
% histogram(distArray(401:500,3), 11)
% title('Microphone 3 85 cm (182.5)')
% 
% % 6. row
% nexttile
% histogram(distArray(501:600,1), 11)
% title('Microphone 1 95 cm')
% nexttile
% histogram(distArray(501:600,2), 11)
% title('Microphone 2 95 cm')
% nexttile
% histogram(distArray(501:600,3), 11)
% title('Microphone 3 95 cm')
% 
% % 7. row
% nexttile
% histogram(distArray(601:700,1), 11)
% title('Microphone 1 105 cm')
% nexttile
% histogram(distArray(601:700,2), 11)
% title('Microphone 2 105 cm')
% nexttile
% histogram(distArray(601:700,3), 11)
% title('Microphone 3 105 cm')
% 
% % 8. row
% nexttile
% histogram(distArray(701:800,1), 11)
% title('Microphone 1 115 cm')
% nexttile
% histogram(distArray(701:800,2), 11)
% title('Microphone 2 115 cm')
% nexttile
% histogram(distArray(701:800,3), 11)
% title('Microphone 3 115 cm')
% 
% % 9. row
% nexttile
% histogram(distArray(801:900,1), 11)
% title('Microphone 1 125 cm')
% nexttile
% histogram(distArray(801:900,2), 11)
% title('Microphone 2 125 cm')
% nexttile
% histogram(distArray(801:900,3), 11)
% title('Microphone 3 125 cm')
% 
% % 10. row
% nexttile
% histogram(distArray(901:1000,1), 11)
% title('Microphone 1 135 cm')
% nexttile
% histogram(distArray(901:1000,2), 11)
% title('Microphone 2 135 cm')
% nexttile
% histogram(distArray(901:1000,3), 11)
% title('Microphone 3 135 cm')