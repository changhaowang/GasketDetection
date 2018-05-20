% Created by Changhao Wang
% Shanghai Jiao Tong University
% Updated in 2018/05/14
function [ center_x, center_y,radius ] = sizedetection( Location )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
sum_x = 0;sum_x2 = 0;sum_x3 = 0;sum_xy = 0;
sum_y = 0;sum_y2 = 0;sum_y3 = 0;sum_x1y2 = 0;sum_x2y1 = 0;
N = length(Location);
for i = 1:N
    sum_x = sum_x + Location(i,1);
    sum_y = sum_y + Location(i,2);
    sum_x2 = sum_x2 + Location(i,1)^2;
    sum_y2 = sum_y2 + Location(i,2)^2;
    sum_x3 = sum_x3 + Location(i,1)^3;
    sum_y3 = sum_y3 + Location(i,2)^3;
    sum_xy = sum_xy + Location(i,1) * Location(i,2);
    sum_x1y2 = sum_x1y2 + Location(i,1) * Location(i,2)^2;
    sum_x2y1 = sum_x2y1 + Location(i,1)^2 * Location(i,2);
end
C = N * sum_x2 - sum_x * sum_x;
D = N * sum_xy - sum_x * sum_y;
E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
G = N * sum_y2 - sum_y * sum_y;
H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
a = (H * D - E * G) / (C * G - D * D);
b = (H * C - E * D) / (D * D - G * C);
c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
center_x = a / (-2);
center_y = b / (-2);
radius = sqrt(a * a + b * b - 4 * c) / 2;
end

