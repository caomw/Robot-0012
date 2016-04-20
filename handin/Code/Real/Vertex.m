classdef Vertex < handle
    properties
        isRobot
        mapPos
        isTarget
        coordinates
        number
        ID
        
        neighbours
        F_cost
        G_cost
        H_cost
        closed
        cameFrom
        
        
    end
    methods
        function obj=Vertex(cord,pos,ID)
            if  nargin == 0
                obj.mapPos = -2;
                obj.coordinates = [0 0];   
                obj.ID=0;
            else
                obj.mapPos=pos;
                obj.coordinates=cord;
                obj.ID=ID;
            end
            obj.neighbours=[];
            
        end
        function addNeighbour(obj,nVert)
           nInd=nVert.ID;
           ind=find(obj.neighbours==nInd);
           if isempty(ind)
               obj.neighbours=[nInd obj.neighbours];
               obj.neighbours=sort(obj.neighbours);
           end
        end
    end
end
        
    
    

