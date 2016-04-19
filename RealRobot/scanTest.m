%robot.turnSensor(0)
clear knownPoints1 knownPoints2
%scan=robot.ultraScan;
data=csvread(['scanresult_' num2str(i) '.txt']);
%scan=data;
data(:,1)=deg2rad(data(:,1));
scan=condData(data(:,1),data(:,2));   

unitV=[0;1];
%% stuff
for i=1:size(scan,1)
    knownPoints1(:,i)=scan(i,3)* Rot((scan(i,1))) *unitV;
    %knownPoints=[knownPoints botScan(i)*Rot(scanLines(i)-pi/2)*unitV];
end
for i=1:size(data,1)
    knownPoints2(:,i)=data(i,2)* Rot((data(i,1))) *unitV;
end

figure
plot(knownPoints1(1,:),knownPoints1(2,:),'x')
hold on
plot(knownPoints2(1,:),knownPoints2(2,:),'.')
axis equal