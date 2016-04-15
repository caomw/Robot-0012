
%data(:,1)=deg2rad(data(:,1));
clear knownPoints
scan=robot.ultraScan();
%condData2(data);   

unitV=[0;1];
%% stuff
for i=1:size(scan,1)
    knownPoints{1}(:,i)=scan(i,2)* Rot((scan(i,1))) *unitV;
    %knownPoints=[knownPoints botScan(i)*Rot(scanLines(i)-pi/2)*unitV];
end

for i=1:size(scan,1)
    knownPoints{2}(:,i)=scan(i,3)* Rot((scan(i,1))) *unitV;
end

figure
plot(knownPoints{1}(1,:),knownPoints{1}(2,:),'x')
hold on
plot(knownPoints{2}(1,:),knownPoints{2}(2,:),'.')
axis equal
grid on