classdef Graph2 < handle
    properties
        vertices
        edges
        validEdges
        coordinates
        map
        originalMap
        mapSize
        n
        maxEdges
        targetID
        startID
        optimalPath
        
        %openSet
        %closedSet
        
    end
    methods
        function obj=Graph2(start,target,map, originalMap)
            obj.map=map;
            obj.originalMap = originalMap;
            obj.mapSize=size(map,1);
            %obj.vertices(size(map,1)+2,1)=Vertex();
            obj.n=size(obj.map,1)+2;
            obj.targetID=obj.n;
            obj.startID=obj.n-1;
            obj.vertices=Vertex.empty(obj.n,0);
            %obj.vertices=cell(obj.n);
            for i=obj.n-2:-1:1
                obj.vertices(i)=Vertex(map(i,:),i,i);
            end
            obj.vertices(end+1)=Vertex(start,-1,obj.startID);
            obj.vertices(end+1)=Vertex(target,0,obj.targetID);
            
            
            obj.maxEdges=obj.n*(obj.n-1)/2;
            obj.edges=Edge.empty(obj.maxEdges,0);
            obj.validEdges=Edge.empty();
            %obj.edges(obj.maxEdges,1)=Edge;
            k=0;
            for i=1:obj.n
                for j=(i+1):obj.n
                    k=k+1;
                    obj.edges(k)=Edge(obj.vertices(i),obj.vertices(j),obj.map);
                end
            end
            
        end
        
        function coord=edgeCoord(obj)
            coord=[obj.vertices(1).coordinates obj.vertices(2).coordinates];
        end
        
        function validateEdges(obj)
            for i=1:obj.maxEdges
                coords=obj.edges(i).coordinates;
                [xi, ~] = polyxpoly(obj.map(:,1), obj.map(:,2), [coords(1) coords(3)],[coords(2) coords(4)]);
                mapPos=obj.edges(i).vertices(1).mapPos;
                if isempty(xi)
                    obj.edges(i).validate();
                    obj.validEdges=[obj.validEdges obj.edges(i)];
                    
                elseif and(or(xi==coords(1),xi==coords(3)),or(size(xi,1)==2,size(xi,1)==1))
                    if mapPos>0
                        d=.1;
                        angle=obj.edges(i).angle;
                        testP=coords(1:2)+[d*cos(angle) d*sin(angle)];
                        in=inpolygon(testP(1),testP(2),obj.map(:,1),obj.map(:,2));
                        if in
                            obj.edges(i).validate();
                            obj.validEdges=[obj.validEdges obj.edges(i)];
                            
                        else
                            %obj.edges(i).valid=0;
                        end
                    end
                end
            end
        
        end
        
        function pathVert=findPath(obj)
            obj.validateEdges();
            closedSet=[];
            openSet=[obj.startID];
            for i=1:obj.n
                obj.vertices(i).G_cost=Inf;
                obj.vertices(i).F_cost=Inf;% distance(obj.vertices(i).coordinates,obj.vertices(obj.targetID).coordinates);
            end
            obj.vertices(obj.startID).G_cost=0;
            obj.vertices(obj.startID).F_cost=distance(obj.vertices(obj.startID).coordinates,obj.vertices(obj.targetID).coordinates);
            k=0;
            while ~isempty(openSet)
                %k=k+1
                F_costs=Inf*ones(numel(openSet),1);
                for i=1:numel(openSet)
                    F_costs(i)=obj.vertices(openSet(i)).F_cost;
                end
                [~,bestOne]=min(F_costs);
                current=obj.vertices(openSet(bestOne));
                if current.ID==obj.targetID
                    %display('done');
                    pathVert=obj.reconstructPath(current);
                    obj.optimalPath=pathVert;
                    return
                end
                
                openSet = openSet(openSet~=current.ID);
                closedSet=[closedSet current.ID];
                
                for i=1:numel(current.neighbours)
                    
                    if any(current.neighbours(i)==closedSet)
                        continue %Ignore the neighbor which is already evaluated.
                    end
                    
                    %The distance from start to goal passing through current and the neighbor.
                    tentative_G_cost = current.G_cost + distance(current.coordinates, obj.vertices(current.neighbours(i)).coordinates);
                    
                    if ~any(current.neighbours(i)==openSet) %Discover a new node
                        openSet=[openSet current.neighbours(i)];
                    elseif tentative_G_cost >= obj.vertices(current.neighbours(i)).G_cost
                        continue		%This is not a better path.
                    end
                    
                    %This path is the best until now. Record it!
                    obj.vertices(current.neighbours(i)).cameFrom = current.ID;
                    obj.vertices(current.neighbours(i)).G_cost = tentative_G_cost;
                    obj.vertices(current.neighbours(i)).F_cost = obj.vertices(current.neighbours(i)).G_cost + distance(obj.vertices(current.neighbours(i)).coordinates, obj.vertices(obj.targetID).coordinates);
                end
            end
            if inpolygon(obj.vertices(obj.startID).coordinates(:,1),obj.vertices(obj.startID).coordinates(:,2),obj.originalMap(:,1),obj.originalMap(:,2))
                if inpolygon(obj.vertices(obj.startID).coordinates(:,1),obj.vertices(obj.startID).coordinates(:,2),obj.map(:,1),obj.map(:,2))
                    %display('I am inside BORDER map and ORIGINAL map I should keep working\n')
                    pathVert=[];
                else 
                    %display('Finding path...')
                    closest = [Inf 0];
                    for i=1:size(obj.map)
                        dist = distance(current.coordinates, obj.map(i,:));
                        if (closest(1) > dist)    
                            closest(1) = dist;
                            closest(2) = i;
                        end
                    end
                    current.neighbours = closest(2);
                    if(distance(current.coordinates, obj.vertices(current.neighbours).coordinates) < 5)
                        %display('is pretty close move next')
                        newNeighbours = obj.vertices(current.neighbours).neighbours;
                        closest(1) = Inf;
                        closest(2) = 1;
                        for i=1:numel(newNeighbours)
                            %closest neighbour to the final target

                            F_cost = distance(obj.vertices(newNeighbours(i)).coordinates, obj.vertices(obj.targetID).coordinates);
                            
                            if closest(1) > F_cost
                                closest(1) = F_cost;
                                closest(2) = i;
                            end
                        end
                        current.G_cost = current.G_cost + distance(current.coordinates, obj.vertices(newNeighbours(closest(2))).coordinates);
                        current.F_cost = obj.vertices(newNeighbours(closest(2))).G_cost + distance(obj.vertices(newNeighbours(closest(2))).coordinates, obj.vertices(obj.targetID).coordinates);
                        pathVert=[current obj.vertices(newNeighbours(closest(2)))];
                    else
                        current.G_cost = current.G_cost + distance(current.coordinates, obj.vertices(current.neighbours).coordinates);
                        current.F_cost = obj.vertices(current.neighbours).G_cost + distance(obj.vertices(current.neighbours).coordinates, obj.vertices(obj.targetID).coordinates);
                        pathVert=[];                           
                    end

                end  
            else
                if inpolygon(obj.vertices(obj.startID).coordinates(:,1),obj.vertices(obj.startID).coordinates(:,2),obj.map(:,1),obj.map(:,2))
                       display('I am inside BORDER map and outside ORIGINAL map that is impossible') 
                       display('Not feasible')
                       pathVert=[];% obj.vertices(obj.startID).coordinates;
                else
                   display('I am outside BORDER map and outside ORIGINAL map, I am lost')
                   display('Not feasible')
                   pathVert=[];% obj.vertices(obj.startID).coordinates;
                end  
            end            
        end
        
        function pathVert=reconstructPath(obj,current)
            pathVert=current;
            while ~(current.ID==obj.startID)
                current = obj.vertices(current.cameFrom);
                pathVert=[current pathVert];
            end
        end
        
        function plotEdges(obj)
            figure(1)
            for i=1:obj.maxEdges
                coords=obj.edges(i).coordinates;
                if obj.edges(i).valid==1
                    plot(coords(1:2:end),coords(2:2:end),'g');
                    hold on
                else
                    %plot(coords(1:2:end),coords(2:2:end),'r');
                    hold on
                end
            end
            
            coords=obj.vertices(obj.n-1).coordinates;
            plot(coords(1),coords(2),'xb');
            coords=obj.vertices(obj.n).coordinates;
            plot(coords(1),coords(2),'xr');
            
            for i=1:numel(obj.optimalPath)-1
                coords=[obj.optimalPath(i).coordinates obj.optimalPath(i+1).coordinates];
                plot(coords(1:2:end),coords(2:2:end),'b');
            end
            %obj.vertices(i).neighbours;
            
        end
            
    end
end
        
    
    

