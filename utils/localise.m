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
 samples = 10; % number of beams
 scanLines= startAngle:abs(endAngle-startAngle)/samples:endAngle;%-abs(startAngle-endAngle)/samples
 scanConfig =  cat(1,cos(scanLines), sin(scanLines))'*30;
 botSim.setScanConfig(scanConfig);           
%botSim.setScanConfig(botSim.generateScanConfig(8));


% generate some random particles inside the map
num = 500; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
isPFLdone = 0;
botEstimate = BotSim(modifiedMap);  %sets up botSim object with adminKey
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    %particles(i).setScanConfig(scanLines,scanOffSet); % scan configuration for each particle
    %particles(i).setScanConfig(particles(i).generateScanConfig(8));
    particles(i).setScanConfig(scanConfig);
    
    % set noise for particles
    particles(i).setMotionNoise(1);
    particles(i).setTurningNoise(0.1);
end

%% Localisation code
maxNumOfIterations = Inf;
n = 0;
converged =0; %The filter has not converged yet
onTheWay=0;
d=Inf;
moveRes=5;
stepSize=3;
reLoc=4;
steps=0;
targetRand=target;
knownPoints=NaN([2 2000],'double');
%[];
beenThere=knownPoints;
knownPoints2=knownPoints;
beenThere2=knownPoints;
unitV=[0;1];
direction=0;
    while(d>stepSize && n < maxNumOfIterations) %%particle filter loop
        %tic
        n = n+1; %increment the current number of iterations
        botScan = botSim.ultraScan(); %get a scan from the real robot.

        %% Write code for updating your particles scans
        [pose, isPFLdone] = PFL( botScan, particles, isPFLdone );
        %pose=[0 0 0];
        botEstimate.setBotPos(pose(1:2));
        botEstimate.setBotAng(pose(3));
        for i=1:numel(botScan)
            knownPoints(:,i)=botScan(i)*Rot(scanLines(i)-pi/2)*unitV;
            %knownPoints=[knownPoints botScan(i)*Rot(scanLines(i)-pi/2)*unitV];
        end

        knownPoints = circshift(knownPoints,numel(botScan),2);

        %% Write code to check for convergence   
        %isPFLdone=0;
        if and(isPFLdone,~onTheWay)
            display('Plan')
            pose(3)=mod(pose(3),2*pi);
            %{
            while pose(3)>2*pi
                pose(3)=pose(3)-2*pi;
            end
            %}

            commands=pathPlan(pose,target,map);
            onTheWay=1;
        elseif ~onTheWay
            display('Explore')

            directionNew=pathExplore(knownPoints,beenThere);

            e=0.5;
            commands(2)=e*directionNew+(1-e)*direction;
            direction=directionNew;
            commands(1)=5;
            onTheWay=1;
        end
        %knownPoints=[knownPoints [0;0]];
        beenThere(:,1)=[0;0];
        beenThere = circshift(beenThere,1,2);

        %% Drawing
        %only draw if you are in debug mode or it will be slow during marking
        %tic
        if botSim.debug()
            figure(1)
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            botSim.drawScanConfig();
            botEstimate.drawBot(30,'b'); %draw robot with line length 30 and green
            
            drawParticles=0;
            if drawParticles
                for i =1:num
                    particles(i).drawBot(3); %draw particle with line length 3 and default color
                end
            end
            plot(target(1),target(2),'xr')
            

            knownPoints2=(knownPoints'*Rot(botSim.getBotAng())')';
            knownPoints2=(knownPoints2'+ones(size(knownPoints2))'*diag(botSim.getBotPos()))';

            beenThere2=(beenThere'*Rot(botSim.getBotAng())')';
            beenThere2=(beenThere2'+ones(size(beenThere2))'*diag(botSim.getBotPos()))';
            %plot(knownPoints(1,:),knownPoints(2,:),'xg')
            plot(knownPoints2(1,:),knownPoints2(2,:),'xb')
            plot(beenThere2(1,:),beenThere2(2,:),'.b')
            %bound=boundary(knownPoints2',1);
            %plot(knownPoints2(1,bound),knownPoints2(2,bound));
            drawnow;
        end
        %toc

        if onTheWay
            if steps==0
                move=commands(1,1)/moveRes;
                turn=commands(1,2);
                steps=steps+1;
                %{
                for i=1:size(knownPoints,2)
                    knownPoints(:,i)=Rot(-turn)*knownPoints(:,i);
                    knownPoints(:,i)=knownPoints(:,i)-[move;0];
                end
                for i=1:size(beenThere,2)
                    beenThere(:,i)=Rot(-turn)*beenThere(:,i);
                    beenThere(:,i)=beenThere(:,i)-[move;0];
                end
                %}

                knownPoints=(knownPoints'*Rot(-turn)')';
                knownPoints=(knownPoints'-ones(size(knownPoints))'*diag([move;0]))';

                beenThere=(beenThere'*Rot(-turn)')';
                beenThere=(beenThere'-ones(size(beenThere))'*diag([move;0]))';

                %commands=[];%commands(2:end,:);
            %elseif and(steps<reLoc,distance(pose(1:2),target)<d);
            elseif steps<reLoc
                move=commands(1,1)/moveRes;
                
                turn=0;
                steps=steps+1;
                %{
                for i=1:size(knownPoints,2)
                    %knownPoints(:,i)=Rot(turn)*knownPoints(:,i);
                    knownPoints(:,i)=knownPoints(:,i)-[move;0];
                end
                for i=1:size(beenThere,2)
                    %knownPoints(:,i)=Rot(turn)*knownPoints(:,i);
                    beenThere(:,i)=beenThere(:,i)-[move;0];
                end
                %}
                %knownPoints=(knownPoints'*Rot(-turn)')';
                knownPoints=(knownPoints'-ones(size(knownPoints))'*diag([move;0]))';

                %beenThere=(beenThere'*Rot(-turn)')';
                beenThere=(beenThere'-ones(size(beenThere))'*diag([move;0]))';

            else
                turn=0;
                move=0;
                steps=0;
                onTheWay=0;
            end
        else
            %% Write code to decide how to move next
            % here they just turn in cicles as an example
            %turn = 0.5;
            %move = 2;
        end




        %% Move
        botSim.turn(turn); %turn the real robot.  
        botSim.move(move); %move the real robot. These movements are recorded for marking 
        for i =1:num %for all the particles. 
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot

        end
        botEstimate.turn(turn);
        botEstimate.move(move);

        % check if robot is inside the map
        if particles(i).insideMap() == 0
            particles(i).randomPose(0);
        end


        d=distance(pose(1:2),target);
        %toc
    end
end
