% Created by Changhao Wang
% Shanghai Jiao Tong University
% Updated in 2018/05/14
classdef Gasket
    properties
        points
        center
        radius 
        normal
    end
    methods
        function obj = Gasket(points,center,radius,normal)
            obj.points = points;
            obj.center = center;
            obj.radius = radius;
            obj.normal = normal;            
        end
    end
end