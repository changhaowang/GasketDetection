% Created by Changhao Wang
% Shanghai Jiao Tong University
% Updated in 2018/05/19+
function [ gasket_radius ] = radiusdetection( radius,num_object )
%UNTITLED2 Summary of this function goes here
%   Obtain the size of gaskets (M18, M22)
% M22 radius = 15.5 mm
% M18 raidus = 12.0 mm
if num_object >= 3
    if radius > 0.011
        gasket_radius = 0.022;
    else
        gasket_radius = 0.018;
    end
else
    if radius > 0.0135
        gasket_radius = 0.022;
    else
        gasket_radius = 0.018;
    end
end
end

