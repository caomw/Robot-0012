function [moveCommands turnCommands] = pathPlan(start,target,map)
%Worst pathplanning function ever.  Assumes there are no obstacles and
%generates random movment instructions

    %x=map(:,1);
    %y=map(:,2);
    %tri=delaunay(map(:,1),map(:,2));
    %triplot(tri,x,y)
    %trisurf(tri,x,y,z);

    tic
    graph=Graph(start,target,map);
    
    %botSim.drawMap();
    
    
    graph.findPath();
    toc
    graph.plotEdges
    print('visibility1','-dsvg')
    

    %% Path planning code
    
    
    %{
    OPEN = priority queue containing START
    CLOSED = empty set
    while lowest rank in OPEN is not the GOAL:
      current = remove lowest rank item from OPEN
      add current to CLOSED
      for neighbors of current:
        cost = g(current) + movementcost(current, neighbor)
        if neighbor in OPEN and cost less than g(neighbor):
          remove neighbor from OPEN, because new path is better
        if neighbor in CLOSED and cost less than g(neighbor): ⁽²⁾
          remove neighbor from CLOSED
        if neighbor not in OPEN and neighbor not in CLOSED:
          set g(neighbor) to cost
          add neighbor to OPEN
          set priority queue rank to g(neighbor) + h(neighbor)
          set neighbor's parent to current

    reconstruct reverse path from goal to start
    by following parent pointers
    %}






    %{
    numOfMoves = 5;
    moveCommands = zeros(1,numOfMoves);
    turnCommands = zeros(1,numOfMoves);
    for i = 1:numOfMoves
       moveCommands(i) = rand(1)*10; 
       turnCommands(i) = rand(1)*pi/2; 
    end
    %}

end