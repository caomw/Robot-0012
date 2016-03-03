function [ output_args ] = cameraSnapshots( cam )
%CAMERASNAPSHOTS Summary of this function goes here
%   Detailed explanation goes here


%cam = webcam

n=12;
for i=11:n
    img = snapshot(cam);
    imwrite(img,['snapshots/webcam_' num2str(i) '.png'])
    x = input('press enter');
end


%clear('cam');
end

