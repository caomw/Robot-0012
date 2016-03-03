map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
x=map(:,1);
y=map(:,2);
plot(x,y,'.','markersize',12);
hold on
plot(x,y);
grid on
tri = delaunay(x,y);
hold on, triplot(tri,x,y), hold off
