function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

    %% setup code
    %you can modify the map to take account of your robots configuration space
    wallDistlim=6;
    modifiedMap = MapBorder2(map, wallDistlim); 
    botSim.setMap(map);
    botDummy=BotSim(modifiedMap);

    %% Scan configuration: 180 degrees vision
    startAngle =-pi/2;
    endAngle = pi/2;
    samples = 10; % number of beams
    scanLines= startAngle:abs(endAngle-startAngle)/samples:endAngle;%-abs(startAngle-endAngle)/samples
    scanConfig =  cat(1,cos(scanLines), sin(scanLines))'*30;
    botSim.setScanConfig(scanConfig);           
    %botSim.setScanConfig(botSim.generateScanConfig(8));


    % generate some random particles inside the map
    num = 100; % number of particles
    particles(num,1) = BotSim; %how to set up a vector of objects
    isPFLdone = 0;
    botEstimate = BotSim(map);  %sets up botSim object with adminKey
    botEstimate.setScanConfig(scanConfig);
    for i = 1:num
        particles(i) = BotSim(map);  %each particle should use the same map as the botSim object
        particles(i).randomPose(0); %spawn the particles in random locations
        %particles(i).setScanConfig(scanLines,scanOffSet); % scan configuration for each particle
        %particles(i).setScanConfig(particles(i).generateScanConfig(8));
        particles(i).setScanConfig(scanConfig);

        % set noise for particles
        particles(i).setMotionNoise(1);
        particles(i).setTurningNoise(0.1);
        
        %drawP{i} = line( 'color','g', 'LineStyle','none','LineWidth',1,'Marker','x','MarkerSize',10,'erase','xor','xdata',particles(i).getBotPos(1),'ydata',particles(i).getBotPos(2));
    end
    %h = animatedline('Color','g','LineStyle','none','Marker','x','MaximumNumPoints',num);

    %% Localisation code
    maxNumOfIterations = Inf;
    n = 0;
    plan=1;
    d=Inf;
    moveRes=5;
    stepSize=5;
    reLoc=3;
    exploreSteps=2;
    steps=0;

    knownPoints=NaN([2 2000],'double');
    beenThere=knownPoints;
    knownPoints2=knownPoints;
    beenThere2=knownPoints;
    unitV=[0;1];
    direction=0;
    robotCommand=zeros(1,2);
    dlim=2;
    done=0;
    while(~done) %%particle filter loop
        %tic
        n = n+1; %increment the current number of iterations
        botScan = botSim.ultraScan(); %get a scan from the real robot.
        
        %%Fitting & Resampling - example
        botScan = fitting(botScan);
        
        [nearest,nidx]=min(botScan);
        %% Write code for updating your particles scans
        [pose, isPFLdone] = PFL( botScan, particles, isPFLdone );
        %[pose, isPFLdone] = PFL2( botScan, particles, isPFLdone, botEstimate );
        %pose=[0 0 0];
        %pose
        botEstimate.setBotPos(pose(1:2));
        botEstimate.setBotAng(pose(3));
        for i=1:numel(botScan)
            knownPoints(:,i)=botScan(i)* Rot(scanLines(i)-pi/2) *unitV;
            %knownPoints=[knownPoints botScan(i)*Rot(scanLines(i)-pi/2)*unitV];
        end

        knownPoints = circshift(knownPoints,numel(botScan),2);

        %% Write code to check for convergence   
        %isPFLdone=0;
        gotoTarget=isPFLdone;
        explore=~isPFLdone;
        
        if gotoTarget && plan
            %display('Target plan')
            
            %pose(3)=mod(pose(3),pi);
            %{
            while pose(3)>2*pi
                pose(3)=pose(3)-2*pi;
            end
            %}
            commands=pathPlan(pose,target,modifiedMap, map);
            if ~isempty(commands)
                robotCommand=commands(1,:);
                direction=robotCommand(2);
            else
                robotCommand(2)=0;%e*directionNew+(1-e)*direction
                direction=directionNew;
                robotCommand(1)=4;
            end
            plan=0;
            steps=0;
            nextPdist=robotCommand(1);
        elseif explore && plan
            %display('Explore plan')

            directionNew=pathExplore(knownPoints,beenThere);

            e=0.3;
            robotCommand(2)=e*directionNew;
            %direction=directionNew;
            robotCommand(1)=stepSize;

            plan=0;
            steps=0;
            nextPdist=robotCommand(1);
        else
            %display('Just go')
        end
            
        %robotCommand
        beenThere(:,1)=[0;0];
        beenThere = circshift(beenThere,1,2);

        %% Drawing
        %only draw if you are in debug mode or it will be slow during marking
        %tic
        if botSim.debug()
            figure(1)
            hold off; %the drawMap() function will clear the drawing when hold is off
            botDummy.drawMap();
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            botSim.drawScanConfig();
            botEstimate.drawBot(30,'b'); %draw robot with line length 30 and green
            
            drawParticles=1;
            if drawParticles
                for i =1:num
                    particles(i).drawBot(3); %draw particle with line length 3 and default color
                    %set(drawP{i},'xdata',particles(i).getBotPos(1),'ydata',particles(i).getBotPos(2));
                    %addpoints(h,particles(i).getBotPos(1),particles(i).getBotPos(2));
                end
                %h.Visible='on';
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

        
        
        
        %display(['Just go ',num2str(steps)]);
        if steps==0
            turn=robotCommand(1,2);
        else
            turn=0;
        end

        if steps==reLoc
            plan=1;
        end
            
        

        if nextPdist>stepSize
            move=stepSize;%robotCommand(1)/moveRes;
        else
            move=nextPdist;
            plan=1;
        end
        nextPdist=nextPdist-move;
        
        nearestNext=nearest-move*cos(scanLines(nidx)+turn);
        if nearestNext<wallDistlim*0.8
            if abs(scanLines(nidx))<0.1
                turn=0.9*pi;
            else
                turn=-2*sign(scanLines(nidx))*(pi/2-abs(scanLines(nidx)))+0.05*(rand(1)-0.5)*abs(scanLines(nidx));
            end
            plan=0;
        end

        %turn=commands(1,2);
        steps=steps+1;

        knownPoints=(knownPoints'*Rot(-turn)')';
        knownPoints=(knownPoints'-ones(size(knownPoints))'*diag([move;0]))';

        beenThere=(beenThere'*Rot(-turn)')';
        beenThere=(beenThere'-ones(size(beenThere))'*diag([move;0]))';

        %display(['commands: ',num2str(move),' ',num2str(turn)]);



        %% Move
        botSim.turn(turn); %turn the real robot.  
        botSim.move(move); %move the real robot. These movements are recorded for marking 
        for i =1:num %for all the particles. 
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot
            % check if particles are inside the map
            if particles(i).insideMap() == 0
                particles(i).randomPose(0);
            end
        end
        botEstimate.turn(turn);
        botEstimate.move(move);
        % check if robot is inside the map
        if botEstimate.insideMap() == 0 && isPFLdone
            display('Error: robot outside the map')
            done=1;
        end
        
        if (distance(pose(1:2),target)<dlim && isPFLdone) || toc>100
            done=1;
        end
        
        
        
        %toc
    end
end
