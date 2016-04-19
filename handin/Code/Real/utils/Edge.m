classdef Edge < handle
    properties
        vertices
        coordinates
        map
        angle
        valid
    end
    methods
        function obj=Edge(vert1,vert2,map)
            if  nargin == 0
                obj.vertices=[Vertex Vertex];
                obj.map=[];
            else
                obj.vertices=[vert1 vert2];
                obj.map=map;
            end
            obj.coordinates=[obj.vertices(1).coordinates obj.vertices(2).coordinates];
            obj.angle=atan2(obj.coordinates(4)-obj.coordinates(2),obj.coordinates(3)-obj.coordinates(1));
            obj.valid=0;
        end
        function coord=edgeCoord(obj)
            coord=[obj.vertices(1).coordinates obj.vertices(2).coordinates];
        end
        function validate(obj)
            obj.valid=1;
            obj.vertices(1).addNeighbour(obj.vertices(2));
            obj.vertices(2).addNeighbour(obj.vertices(1));
        end
 
    end
end
        
    
    

