function [ img1t ] = getDisp( img1,img2,t )
%GETDISP Summary of this function goes here
%   Detailed explanation goes here


for i=1:3
    img1t(:,:,i) = im2bw(img1(:,:,i),t);
    figure
    imshow(img1t(:,:,i));
end



end

