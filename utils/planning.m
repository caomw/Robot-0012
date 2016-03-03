clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map


map=[0 0;120 0];
h=10;
d=100;
f=3;
n=5
for i=1:n
    map=[map;map(end,:)+[0 h]];
    map=[map;map(end,:)+[-d 0]];
    map=[map;map(end,:)+[0 h]];
    map=[map;map(end,:)+[d 0]];
end
map=[map;map(end,:)+[0 h]];
map=[map;map(end,:)+[-120 0]];
map=[map;map(end,:)+[0 -h/f]];
for i=1:n
    map=[map;map(end,:)+[d 0]];
    map=[map;map(end,:)+[0 -h/f]];
    map=[map;map(end,:)+[-d 0]];
    map=[map;map(end,:)+[0 -(2*f-1)*h/f]];
end

    

%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
%Predefined start and target positions
    
start=[110 10];
target=[110 110];
startPosition =  [20,20,70]; %These will change
%targetPositions = [80,80;100,20;230,70]; %These will change
debug=0;
% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,startPosition,debug);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;
%botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
%target = botSim.getRndPtInMap(10);  %gets random target.

tic %starts timer

%your localisation function is called here.
%returnedBot = localise(botSim,map,target); %Where the magic happens
[moveCommands turnCommands]=pathPlan(start,target,map);

resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos(debug))