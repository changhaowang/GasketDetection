% Created by Changhao Wang,
% Shanghai Jiao Tong University,
% Updated in 2018/05/15
function [cluster,ptCloud_remain] =regionGrow(ptCloud,index,radius)
num =1;
Location = ptCloud.Location;
Color = ptCloud.Color;
len = length(ptCloud);
cluster_location = zeros(len,3);
cluster_color = zeros(len,3);

% get seed information
seed_location = ptCloud.Location(index,:);
seed_r = ptCloud.Color(index,1);
seed_g = ptCloud.Color(index,2);
seed_b = ptCloud.Color(index,3);
cluster_location(num,:) = seed_location;
cluster_color(num,:) = [seed_r,seed_g,seed_b];
% remove the seed from ptCloud_remain
%ptCloud_remain.Location(index,:) = [];
%ptCloud_remain.Color(index,:) = [];
Location(index,:) = [];
Color(index,:) = [];
% use KDtree to search nearest point
%kdtree = KDTreeSearcher(ptCloud.Location);
%ind = rangesearch(kdtree,seed_location,radius);%%%%%%%%
%count = length(ind{1});
%serachpoint = seed_location;
for i = 1:len
    % build a kdtree
    if cluster_location(i,:) ~= [0,0,0]
        kdtree = KDTreeSearcher(Location);
        idx = rangesearch(kdtree,cluster_location(i,:),radius);
        count = length(idx{1});
        idx = sort(idx{1});
        for j = 1:count
            if (Color(idx(count+1-j),1) - seed_r < 100) && ...
                    (Color(idx(count+1-j),2) - seed_g < 100) && ...
                    (Color(idx(count+1-j),3) - seed_b < 100)
                num = num + 1;
                cluster_location(num,:) = Location(idx(count+1-j),:);
                cluster_color(num,:) = Color(idx(count+1-j),:);
            end
            Location(idx(count+1-j),:) = [];
            Color(idx(count+1-j),:) = [];
        end
    else
        break;
    end
end
cluster_location = cluster_location(1:num,:);
cluster_index = zeros(num,1);
for i = 1:num
    cluster_index(i) = find(any((ptCloud.Location - repmat(cluster_location(i,:),ptCloud.Count,1)).')==0);
end
cluster_index = sort(cluster_index);

cluster = select(ptCloud,cluster_index);

%calculate ptCloud remain
Location = ptCloud.Location;
for i = 1:num
    Location(cluster_index(num-i+1),:) = [];
end
len = length(Location);
remain_index = zeros(len,1);
for i = 1:len
    remain_index(i) = find(any((ptCloud.Location - repmat(Location(i,:),ptCloud.Count,1)).')==0);
end
ptCloud_remain = select(ptCloud,remain_index);
%index = find()
% cluster = ptCloud;
% cluster.Location = cluster_location;
% cluster.Color = cluster_color;
% cluster.Count = num;
end