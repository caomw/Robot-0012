function [commands] = pathPlan(startPose,target,map)
%Worst pathplanning function ever.  Assumes there are no obstacles and
%generates random movment instructions

    graph=Graph(startPose(1:2),target,map);
    path=graph.findPath();
    
    if size(path)
        commands=zeros(numel(path),2);
        coords1=path(1).coordinates;
        coords2=path(2).coordinates;
        commands(1,1)=distance(coords1,coords2);
        angle=atan2(coords2(2)-coords1(2),coords2(1)-coords1(1));
        commands(1,2)=angle-startPose(3);
        for i=2:numel(path)-1
            coords1=path(i).coordinates;
            coords2=path(i+1).coordinates;
            commands(i,1)=distance(coords1,coords2);
            commands(i,2)=atan2(coords2(2)-coords1(2),coords2(1)-coords1(1))-angle;
            angle=atan2(coords2(2)-coords1(2),coords2(1)-coords1(1));
        end
    else
        commands=[0 0];
    end
end