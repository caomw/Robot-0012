clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.
joy = vrjoystick(1);

botSim.setBotPos([20 40])
botSim.setBotAng(pi)

botPosition = botSim.getBotPos()

botAngle = botSim.getBotAng()
botSim.drawMap();
botSim.drawBot(3);

dt=0.1;
while 1
    tic
    [axes, buttons, povs] = read(joy);
    v=-2*axes(2);
    theta=-axes(1);
    botSim.turn(theta);
    botSim.move(v);
    %insideMap = botSim.insideMap()
    
    hold off;
    botSim.drawMap();
    botSim.drawBot(3);
    drawnow
    %toc
    while toc<=0.1
    end
    toc
end

return


botSim.setMotionNoise(0.1);

for i = 1:50
    botSim.setBotPos([25 10]);
    botSim.move(60);
    botSim.drawBot(3);
end


botSim.setBotPos([25 10]);
botSim.setBotAng(pi/2);

hold off
botSim.drawMap();
botSim.drawBot(3);


botSim.setTurningNoise(0.05);
for i = 1:50
    botSim.setBotPos([25 10]);
    botSim.setBotAng(pi/2);
    botSim.move(60);
    botSim.drawBot(3);
end
disp('Example1 finished');