function [ NewMap ] = MapBorder( Map,normalsize )
%   MapBorder
%   Description: This function returns the new map with the border
%   Input: 
%       1)Map - Receives the original map
%       2)normalsize - is the size of how much the map is going to contract
%   Output:
%       NewMap - returns the coordinates of the new map
%   Example:
%   maptest = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
%   normalsize = 5;
%   NewMapCoordinates = MapBorder(maptest,normalsize);

    TempMap = zeros(size(Map, 1));
    NewMap = zeros(size(Map));
    for idx = 1:size(Map)
        %verify if it's the last point
        if idx == size(Map, 1)
            A = [Map(idx, 1),Map(idx, 2)]; B = [Map(1, 1),Map(1, 2)];
            x = [A(1);B(1)];
            y = [A(2);B(2)];
            normal = [mean(x),mean(y)] + normalsize*null(A-B)';
            %verify the normal is inside the map
            %if ~botSim.pointInsideMap([normal(1) normal(2)])
            if ~inpolygon(normal(1),normal(2),Map(:,1),Map(:,2))
                normal = [mean(x),mean(y)] - normalsize*null(A-B)';
            end
                %The normal is in on Y and not inverted
                if(mean(x) == Map(idx, 1)) && (mean(y) ~= Map(idx, 2))
                     %Declare partial edge
                    TempMap(idx,1) = normal(1);
                    TempMap(idx, 2) = Map(idx, 2);  
                    TempMap(idx, 3) = normal(1);
                    TempMap(idx, 4) = Map(1, 2);

                %The normal is in on X and not inverted
                elseif(mean(y) == Map(idx, 2)) && (mean(x) ~= Map(idx, 1))
                    %Declare partial edge
                    TempMap(idx,1) = Map(idx,1);
                    TempMap(idx, 2) = normal(2);  
                    TempMap(idx, 3) = Map(1, 1);
                    TempMap(idx, 4) = normal(2);
                end
        else
                A = [Map(idx, 1),Map(idx, 2)]; B = [Map(idx+1, 1),Map(idx+1, 2)];
                x = [A(1);B(1)];
                y = [A(2);B(2)];
                normal = [mean(x),mean(y)] + normalsize*null(A-B)';
                inverted = 0;
                %verify the normal is inside the map
                if ~inpolygon(normal(1),normal(2),Map(:,1),Map(:,2))
                    normal = [mean(x),mean(y)] - normalsize*null(A-B)';
                    inverted = 1;
                end

                %The normal is in on Y 
                if(mean(x) == Map(idx, 1)) && (mean(y) ~= Map(idx, 2))
                    %Declare partial edge
                    TempMap(idx,1) = normal(1);
                    TempMap(idx, 2) = Map(idx, 2);  
                    TempMap(idx, 3) = normal(1);
                    TempMap(idx, 4) = Map(idx+1, 2);

                %The normal is in on X and not inverted
                elseif (mean(y) == Map(idx, 2)) && (mean(x) ~= Map(idx, 1))
                    %Declare partial edge
                    TempMap(idx,1) = Map(idx,1);
                    TempMap(idx, 2) = normal(2);  
                    TempMap(idx, 3) = Map(idx+1, 1);
                    TempMap(idx, 4) = normal(2);

                %Slope Line
                elseif (mean(x) ~= Map(idx, 1)) && ~inverted
                    slope = (Map(idx+1, 2)-Map(idx, 2))/(Map(idx+1, 1)-Map(idx, 1));

                    %Declare partial edge
                    TempMap(idx,1) = Map(idx, 1)-normalsize;
                    TempMap(idx, 2) = Map(idx, 2)+normalsize;  
                    TempMap(idx, 3) = Map(idx+1, 1)-normalsize;
                    TempMap(idx, 4) = Map(idx+1, 2)+normalsize;

                %Slope Line
                elseif (mean(x) ~= Map(idx, 1))&& inverted
                    slope = (Map(idx+1, 2)-Map(idx, 2))/(Map(idx+1, 1)-Map(idx, 1));
                    if slope < 0
                        %Declare partial edge
                        TempMap(idx,1) = Map(idx, 1)-normalsize;
                        TempMap(idx, 2) = Map(idx, 2)-normalsize;  
                        TempMap(idx, 3) = Map(idx+1, 1)-normalsize;
                        TempMap(idx, 4) = Map(idx+1, 2)-normalsize;
                    else
                        %Declare partial edge
                        TempMap(idx, 1) = Map(idx, 1)+normalsize;
                        TempMap(idx, 2) = Map(idx, 2)-normalsize;  
                        TempMap(idx, 3) = Map(idx+1, 1)+normalsize;
                        TempMap(idx, 4) = Map(idx+1, 2)-normalsize;
                    end
                end

        end
    end

    for idx = 1:size(Map)
        if idx == size(Map, 1)
            line2 = [TempMap(1,1) TempMap(1,2); TempMap(1,3) TempMap(1,4)];
            line1 = [TempMap(idx,1) TempMap(idx,2); TempMap(idx,3) TempMap(idx,4)];
           [points] = Intersection(line1, line2);
            NewMap(1, 1) = points(1);
            NewMap(1, 2) = points(2);
        else
            line2 = [TempMap(idx+1,1) TempMap(idx+1,2); TempMap(idx+1,3) TempMap(idx+1,4)];
            line1 = [TempMap(idx,1) TempMap(idx,2); TempMap(idx,3) TempMap(idx,4)];
            [points] = Intersection(line1, line2);
            NewMap(idx+1, 1) = points(1);
            NewMap(idx+1, 2) = points(2);
        end
    end
end

