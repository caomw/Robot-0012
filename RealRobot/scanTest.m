%robot.turnSensor(0)
clear knownPoints1 knownPoints2
%scan=robot.ultraScan;
data=csvread(['scanresult_3.txt']);
%scan=data;
scan=condData(data);   
unitV=[0;1];
%% stuff
for i=1:size(scan,1)
    knownPoints1(:,i)=scan(i,3)* Rot((scan(i,1))) *unitV;
    %knownPoints=[knownPoints botScan(i)*Rot(scanLines(i)-pi/2)*unitV];
end
for i=1:size(data,1)
    knownPoints2(:,i)=data(i,2)* Rot(deg2rad(data(i,1))) *unitV;
end

%plot(knownPoints1(1,:),knownPoints1(2,:),'x')
%hold on
plot(knownPoints2(1,:),knownPoints2(2,:),'.')
axis equal