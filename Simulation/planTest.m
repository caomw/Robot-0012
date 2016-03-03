clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

target=[225 75];
botSim.setBotPos([50 75])
botSim.setBotAng(pi)

botPosition = botSim.getBotPos()

botAngle = botSim.getBotAng()
botSim.drawMap();
botSim.drawBot(3);
onTheWay=0;
d=Inf;
maxNumOfIterations=Inf;
n=0;
while(d>1 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    
    
    %% Write code for updating your particles scans
    pose=[botSim.getBotPos() botSim.getBotAng()];
    isPFLdone=1;
    
    %% Write code to check for convergence   
	if and(isPFLdone,~onTheWay)
        display('planning')
        while pose(3)>2*pi
            pose(3)=pose(3)-2*pi;
        end
        
        commands=pathPlan(pose,target,map);
        pose;
        commands;
        onTheWay=1;
    end
        
    if onTheWay
        if size(commands,1)>0
            move=commands(1,1);
            turn=commands(1,2);
            commands=commands(2:end,:);
        else
            onTheWay=0;
        end
    else
        %% Write code to decide how to move next
        % here they just turn in cicles as an example
        turn = 0.5;
        move = 2;
    end
    
    
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    
    
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if 1%botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        %for i =1:num
        %    particles(i).drawBot(3); %draw particle with line length 3 and default color
        %end
        plot(target(1),target(2),'xr')
        drawnow;
    end
    d=distance(pose(1:2),target);
    pause(1)
end