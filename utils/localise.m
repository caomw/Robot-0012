function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%% Scan configuration: 180 degrees vision
startAngle =-pi/2;
endAngle = pi/2;
samples = 13; % number of beams
angles= (startAngle:(endAngle - startAngle)/(samples-1):endAngle);
scanLines =  [cos(angles); sin(angles)]';
scanOffSet = [4 0];
botSim.setScanConfig(scanLines,scanOffSet); % scan configuration for robot

%generate some random particles inside the map
num =500; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
isPFLdone = 0;
botEstimate = BotSim(modifiedMap);  %sets up botSim object with adminKey
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
     particles(i).setScanConfig(scanLines,scanOffSet); % scan configuration for each particle
end

%% Localisation code
maxNumOfIterations = Inf;
n = 0;
converged =0; %The filter has not converged yet
onTheWay=0;
d=Inf;
while(d>1 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    [pose, isPFLdone] = PFL( num, botScan, particles, isPFLdone );
    botEstimate.setBotPos(pose(1:2));
    botEstimate.setBotAng(pose(3));
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
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
        botEstimate.turn(turn);
        botEstimate.move(move);
    end
    
    % check if robot is inside the map
    if particles(i).insideMap() == 0
        particles(i).randomPose(0);
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        botEstimate.drawBot(30,'b'); %draw robot with line length 30 and green
        %for i =1:num
        %    particles(i).drawBot(3); %draw particle with line length 3 and default color
        %end
        plot(target(1),target(2),'xr')
        drawnow;
    end
    d=distance(pose(1:2),target);
end
end
