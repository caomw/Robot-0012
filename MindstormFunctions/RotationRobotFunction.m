function [ theta ] = moveTo( x )
%MOVETO Summary of this function goes here
%   Detailed explanation goes here

R=13/(2*pi);
w=13;
Z=R*[.5 .5; 1/w -1/w];
x(2)=deg2rad(x(2));
theta=Z\x;
theta=rad2deg(theta);
end

