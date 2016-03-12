%maptest = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
normalsize = 5;
%maptest = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80];
maptest = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
botSim = BotSim(maptest);
botSim.drawMap()

for idx = 1:size(maptest)
    %verify if it's the last point
    if idx == size(maptest, 1)
        A = [maptest(idx, 1),maptest(idx, 2)]; B = [maptest(1, 1),maptest(1, 2)];
        x = [A(1);B(1)];
        y = [A(2);B(2)];
        normal = [mean(x),mean(y)] + normalsize*null(A-B)';
        %verify the normal is inside the map
        if ~botSim.pointInsideMap([normal(1) normal(2)])
            normal = [mean(x),mean(y)] - normalsize*null(A-B)';
        end
        %line([mean(x),normal(1)],[mean(y),normal(2)],'color','r','LineWidth',2)
        
            %The normal is in on Y and not inverted
            if(mean(x) == maptest(idx, 1)) && (mean(y) ~= maptest(idx, 2))
                line([normal(1),normal(1)],[maptest(idx, 2),maptest(1, 2)],'color','k','LineWidth',2)
            
            %The normal is in on X and not inverted
            elseif(mean(y) == maptest(idx, 2)) && (mean(x) ~= maptest(idx, 1))
                line([maptest(idx, 1),maptest(1, 1)],[normal(2),normal(2)],'color','k','LineWidth',2)
            end
    else
            A = [maptest(idx, 1),maptest(idx, 2)]; B = [maptest(idx+1, 1),maptest(idx+1, 2)];
            x = [A(1);B(1)];
            y = [A(2);B(2)];
            normal = [mean(x),mean(y)] + normalsize*null(A-B)';
            inverted = 0;
            %verify the normal is inside the map
            if ~botSim.pointInsideMap([normal(1) normal(2)])
                normal = [mean(x),mean(y)] - normalsize*null(A-B)';
                inverted = 1;
            end
        
            %The normal is in on Y 
            if(mean(x) == maptest(idx, 1)) && (mean(y) ~= maptest(idx, 2))
                line([normal(1),normal(1)],[maptest(idx, 2),maptest(idx+1, 2)],'color','m','LineWidth',2)

            %The normal is in on X and not inverted
            elseif (mean(y) == maptest(idx, 2)) && (mean(x) ~= maptest(idx, 1))
                line([maptest(idx, 1),maptest(idx+1, 1)],[normal(2),normal(2)],'color','c','LineWidth',2)

            elseif (mean(x) ~= maptest(idx, 1)) && ~inverted
                slope = (maptest(idx+1, 2)-maptest(idx, 2))/(maptest(idx+1, 1)-maptest(idx, 1));
                fprintf('X1:%d, X2:%d, Y1:%d, Y2:%d, Normal 1:%d, Normal 2:%d, SLOPE:%d\n', maptest(idx, 1), maptest(idx+1, 1),maptest(idx, 2), maptest(idx+1, 2), normal(1), normal(2), slope)
                line([maptest(idx, 1)-normalsize-slope,maptest(idx+1, 1)-normalsize-slope],[maptest(idx, 2)+normalsize+slope,maptest(idx+1, 2)+normalsize+slope],'color','g','LineWidth',2) 

            elseif (mean(x) ~= maptest(idx, 1))&& inverted
                slope = (maptest(idx+1, 2)-maptest(idx, 2))/(maptest(idx+1, 1)-maptest(idx, 1));
                fprintf('X1:%d, X2:%d, Y1:%d, Y2:%d, Normal 1:%d, Normal 2:%d, SLOPE:%d\n', maptest(idx, 1), maptest(idx+1, 1),maptest(idx, 2), maptest(idx+1, 2), normal(1), normal(2), slope)
                if slope < 0
                    line([maptest(idx, 1)-normalsize+slope,maptest(idx+1, 1)-normalsize+slope],[maptest(idx, 2)-normalsize+slope,maptest(idx+1, 2)-normalsize+slope],'color','b','LineWidth',2) 
                else
                    line([maptest(idx, 1)+normalsize+slope,maptest(idx+1, 1)+normalsize+slope],[maptest(idx, 2)-normalsize-slope,maptest(idx+1, 2)-normalsize-slope],'color','k','LineWidth',2) 
                end
            end
        
    end
end
