% Created by Changhao Wang
% Shanghai Jiao Tong University
% Updated in 2018/05/14
classdef Cluster
    properties
        Location
        cluster_index 
        num_cluster
    end
    methods
        function obj = Cluster(Location,cluster_index,num_cluster)
            obj.Location = Location;
            obj.cluster_index = cluster_index;
            obj.num_cluster = num_cluster;
        end
    end
end