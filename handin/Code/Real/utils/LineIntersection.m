function [ intersectionpoint ]  = LineIntersection( line1, line2 )
%   Intersection
%   Description: This function returns the x and y coordinates where two
%   lines instersect
%   Input: 
%       1)line1 - coordinates x1, y1, x2, y2 of line1
%       2)line2 - coordinates x1, y1, x2, y2 of line2
%   Output:
%       intersectionpoint - returns a vector with x and y coordiantes
%   Example:
%       line2 = [x1 y1; x2 y2];
%       line1 = [x1 y1; x2 y2];
%       [points] = Intersection(line1, line2);

slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
intercept = @(line,m) line(1,2) - m*line(1,1);
if line1(2,1) - line1(1,1) == 0
    m2 = slope(line2);
    xintersect = line1(2,1);
    b2 = intercept(line2,m2);
    yintersect = m2*xintersect + b2;
elseif line2(2,1) - line2(1,1) == 0
    m1 = slope(line1);
    xintersect = line2(2,1);
    b1 = intercept(line1,m1);
    yintersect = m1*xintersect + b1;
else
    m1 = slope(line1);
    m2 = slope(line2);
    b1 = intercept(line1,m1);
    b2 = intercept(line2,m2);
    xintersect = (b2-b1)/(m1-m2);
    yintersect = m1*xintersect + b1;
end
intersectionpoint = [xintersect yintersect];
end

