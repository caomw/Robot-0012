classdef BotNode < BotSim
    properties
        G_cost
        H_cost
        F_cost
        open
        closed
        startNode
        targetNode
    end
    
    
    methods
        function obj = BotNode(map,start,target)
            if nargin==0
                map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
            end
            obj@BotSim(map);
            obj.G_cost=0;
            obj.H_cost=0;
            obj.F_cost=0;
            obj.open=0;
            obj.closed=1;
            if nargin>1
                obj.startNode=start;
                obj.targetNode=target;
            end
        end
    end
    %{
    methods
        %Constructor
        function bot = BotNode(newMap,noiseLevel,adminKey)
            if nargin >0
                if nargin <3
                    adminKey =0;
                    if nargin <2
                        noiseLevel = [0 0 0];                        
                    end
                end
                bot.pos = [0 0];
                bot.ang = 0;
                bot.dir = [cos(bot.ang) sin(bot.ang)];
                bot.setMap(newMap);
                bot.scanOffset = [0 0];
                bot.scanConfig = generateScanConfig(bot,6);
                bot.updateScanLines(0,1);
                bot.sensorNoise = noiseLevel(1);  %constant noise model. Error standard deviation in cm
                bot.motionNoise = noiseLevel(2);  %proportional noise model. cm error stdDev per unit length in cm/cm
                bot.turningNoise = noiseLevel(3); %porportional noise model. Radian stdDev error per radian rad/rad
                bot.adminKey = adminKey;
                bot.PosHistory =[];
                bot.MoveCount =1;
            end
        end
    end
    %}
end