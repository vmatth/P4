function plotRoom(setup, vector, grader2test)

    x = setup.room.dimensions(1);
    y = setup.room.dimensions(2);
    z = setup.room.dimensions(3);

    figure(69);
    title('Room');
    xlabel('Distance [m]');ylabel('Distance [m]');

    
    %% Lines
    l = line([0, x], [0, 0]); %Bottom line
    line([0, x], [y, y]); %Top line
    line([0, 0], [0, y]); %Left line
    line([x, x], [0, y]); %Right line
    
    %% Limits
    xlim([-1, x+1]);
    ylim([-1, y+1]);
    grid on
    
    %% Mic & Speaker
    hold on
    color1 = '.b';
    color2 = '.g';
    color3 = '.r';
    color  = [color1; color2; color3]   ; 
    for i=1:setup.array.micNumber
    mic(i) = plot(setup.room.receivPos(i,1), setup.room.receivPos(i,2), color(i,:), 'MarkerSize', 20); %Mic
    end
    speaker = plot(setup.room.sourcePos(1), setup.room.sourcePos(2), '.y', 'MarkerSize', 15); %Speaker
    
    hold on
        x=setup.room.sourcePos(1);
        y=setup.room.sourcePos(2);
        r=0.2;
        th = 0:pi/50:2*pi;
        xunit = r * cos(th) + x;
        yunit = r * sin(th) + y;
        robot = plot(xunit, yunit, 'g');
    hold off
    % dette bruges til at lave en linje i kørselsretningen
        x1=setup.room.receivPos(1,1);
        y1=setup.room.receivPos(1,2);
        x2=setup.room.receivPos(2,1);
        y2=setup.room.receivPos(2,2);
        miclineA = (y2-y1)/(x2-x1);
        miclineB = y1-(miclineA*x1);
        speakerlineA = (-1)/miclineA;
        speakerlineB = setup.room.sourcePos(2) - (speakerlineA * setup.room.sourcePos(1));
        xcrossing = (speakerlineB-miclineB)/(miclineA-speakerlineA);
        ycrossing = miclineA*(xcrossing)+miclineB;
        p1 = [setup.room.sourcePos(1) setup.room.sourcePos(2)];       % First Point
        p2 = [setup.room.receivPos(1,1) setup.room.receivPos(1,2)];                         % Second Point
        dp = p2-p1;   % Difference
        hold on
        quiver(p1(1),p1(2),dp(1),dp(2),1 ,'MaxHeadSize',0.5,'Color','r','LineWidth',2)
        hold on
        plot(vector(1), vector(2),'or')
    legend([l, mic(1) mic(2) mic(3) speaker], {'Wall', 'Mic1', 'Mic2' 'Mic3' 'Speaker'});
%     
     savestr = sprintf('$DOAat_%4.2f_angle.png',grader2test);
     saveas(gcf,savestr);
%     
end