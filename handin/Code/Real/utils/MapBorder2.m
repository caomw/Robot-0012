function [ NewMap ] = MapBorder2( Map,normalsize )
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
    
    TempMap = zeros(size(Map, 1),4);
    NewMap = zeros(size(Map));
    angle=0;
    for idx = 1:size(Map)
        if idx == size(Map, 1)-1
            A = [Map(idx, 1),Map(idx, 2), 0]; B = [Map(idx+1, 1),Map(idx+1, 2), 0];C = [Map(1, 1),Map(1, 2), 0];
        elseif idx == size(Map, 1)
            A = [Map(idx, 1),Map(idx, 2), 0]; B = [Map(1, 1),Map(1, 2), 0];C = [Map(2, 1),Map(2, 2), 0];
        else
            A = [Map(idx, 1),Map(idx, 2), 0]; B = [Map(idx+1, 1),Map(idx+1, 2), 0];C = [Map(idx+2, 1),Map(idx+2, 2), 0];
        end
        v1=B-A;
        v2=C-B;
         x=cross(v1,v2);
         angle =angle+atan2(sign(x(end))*norm(x),dot(v1,v2));
    end
    
    for idx = 1:size(Map)
        %verify if it's the last point
        if idx == size(Map, 1)
            A = [Map(idx, 1),Map(idx, 2)]; B = [Map(1, 1),Map(1, 2)];
        else
            A = [Map(idx, 1),Map(idx, 2)]; B = [Map(idx+1, 1),Map(idx+1, 2)];
        end
        
        x = [A(1);B(1)];
        y = [A(2);B(2)];
        %normal = normalsize*null(A-B)';
        dirV=(B-A)'/norm(A-B);
        normal=sign(angle)*[-dirV(2) dirV(1)];
        midP = [mean(x),mean(y)] + normal;
        A2=A+normalsize*normal;
        B2=B+normalsize*normal;
        
        %if ~inpolygon(midP(1),midP(2),Map(:,1),Map(:,2))
        %    A2=A-normalsize*null(A-B)';
        %    B2=B-normalsize*null(A-B)';
        %end

        TempMap(idx,:)=[A2 B2];

    end

    for idx = 1:size(Map)
        if idx == size(Map, 1)
            line2 = [TempMap(1,1) TempMap(1,2); TempMap(1,3) TempMap(1,4)];
            line1 = [TempMap(idx,1) TempMap(idx,2); TempMap(idx,3) TempMap(idx,4)];
           [points] = LineIntersection(line1, line2);
            NewMap(1, 1) = points(1);
            NewMap(1, 2) = points(2);
        else
            line2 = [TempMap(idx+1,1) TempMap(idx+1,2); TempMap(idx+1,3) TempMap(idx+1,4)];
            line1 = [TempMap(idx,1) TempMap(idx,2); TempMap(idx,3) TempMap(idx,4)];
            [points] = LineIntersection(line1, line2);
            NewMap(idx+1, 1) = points(1);
            NewMap(idx+1, 2) = points(2);
        end
    end
end

