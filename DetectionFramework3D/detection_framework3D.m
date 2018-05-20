% Created by Changhao Wang, 
% Shanghai Jiao Tong University,
% Updated in 2018/05/14
%% Add Path
addpath(genpath('PointCloud'));
addpath(genpath('SpectralClustering'));
addpath(genpath('Class'));
addpath(genpath('RegionGrow'));
addpath(genpath('SizeDetection'));
addpath(genpath('Calibration'));
addpath(genpath('Kmeans'));
%% Camera Calibration
cameraCalibrator;
%% Using Kinect V1 to get Point Cloud
colorDevice = imaq.VideoDevice('kinect',1);
colorDevice.ReturnedDataType = 'uint8'; 
depthDevice = imaq.VideoDevice('kinect',2);
step(colorDevice);
step(depthDevice);
colorImage = step(colorDevice);
depthImage = step(depthDevice);
ptCloud_ori = pcfromkinect(depthDevice,depthImage,colorImage);
pcshow(ptCloud_ori);
%% Point Cloud Transform
load('cameraParams.mat');
% Transform point cloud from camera coordinate to world coordinate
index = 1;
% The t, R definition is quite different from robotics 
% refer to: https://www.mathworks.com/help/vision/ref/extrinsics.html#outputarg_translationVector
t = cameraParams.TranslationVectors(index,:)'./1000;   % unit: m 
R = cameraParams.RotationMatrices(:,:,index);
T_Board2Camera = [[R,-R*t]; [0,0,0,1]];
T_Camera2Kinect = [-1  0  0  0;...
    0 -1  0  0;...
    0  0  1  0;...
    0  0  0  1];
T_World2Board = [ 1  0   0  -0.08;...
    0 -1   0  -1.22;...
    0  0  -1  -0.02;...
    0  0   0  1];
%load('T_W2K.mat');
T_W2K = (T_World2Board*T_Board2Camera*T_Camera2Kinect);
R = T_W2K(1:3,1:3);
t = T_W2K(1:3,4);
T_W2K = [[R;t.'],[0;0;0;1]];
transform = affine3d(T_W2K);
ptCloud = pctransform(ptCloud_ori,transform);
figure;
pcshow(ptCloud);
clear index t R T_W2K T_Board2Camera transform
%% Load Point Cloud
% ptCloud = pcread('M18and22occlu.ply');
% ptCloud = pcread('5_16.ply');
ptCloud_ori = pcread('5_19_ori.ply');
%% Plane segmentation by RANSAC
ptCloud = pcdenoise(ptCloud);
maxDistance = 0.05;
referenceVector = [0,0,1];
maxAngularDistance = 5;
[model,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);
roi = [-inf,inf;0.4,inf;-inf,inf];
sampleIndices = findPointsInROI(ptCloud,roi);
figure;
pcshow(plane);
title('RANSAC fitted plane');
clear maxDistance referenceVector maxAngularDistance inlierIndices outlierIndices roi sampleIndices
%% Object Detection by Color-based filter
mask_rgb = {@(r) (r<=120), @(g) (g<=120), @(b) (b<=120)};
r = plane.Color(:,1);
g = plane.Color(:,2);
b = plane.Color(:,3);
r_mask = mask_rgb{1}(r);
g_mask = mask_rgb{2}(g);
b_mask = mask_rgb{3}(b);
color_mask = r_mask & g_mask & b_mask;
object = select(plane,find(color_mask ~= 0));
pcshow(object);
clear mask_rgb r g b r_mask g_mask b_mask color_mask 
%% Object Segmentation by Region Growing (Color)
% num_cluster = input('Please enter the number of the clusters on the table: ');
object_1 = object;
count = object_1.Count;
% for i = 1:num_cluster
%     Location(:,:,i) = zeros(count,3);
%     Color(:,:,i) = zeros(count,3);
% end
counter = 0;
while object_1.Count > 100
    searchindex = round(object_1.Count.*rand(1,1));
    [cluster,object_1] = regionGrow(object_1,searchindex,0.05);
    if cluster.Count > count./8
        counter = counter + 1;
        Location(1:cluster.Count,:,counter) = cluster.Location;
        Color(1:cluster.Count,:,counter) = cluster.Color;
        figure (counter);
        pcshow(cluster);
    end
end
num_cluster = size(Location,3);
clear object_1 count counter searchindex cluster
%% Object Segmentation by Spectral Clustering
cluster = [];
for i = 1:num_cluster
    data = Location(find(all(Location(:,:,i).')),1:3,i);
    neighbor = 15;
    Type = 1;
    sigma = 1;
    SimGraph = SimGraph_NearestNeighbors(data.', neighbor, Type, sigma);
    k = input('Please enter the Number of objects to look for: ');
    if k == 1
        cluster_index = ones(length(data),1);
        cluster_new = Cluster(data,cluster_index,k);
        cluster = [cluster,cluster_new];
        continue;
    end
    cluster_index = SpectralClustering(SimGraph, k, 2);
    cluster_new = Cluster(data,cluster_index,k);
    cluster = [cluster,cluster_new];
end
clear num_object data neighbor Type sigma SimGraph k cluster_index cluster_new i
%% Size detection (least square)
gasket = [];
Figure_Marker = '*';
Figure_Color = [0 0 0; 0 0 1; 0 1 0; 1 0 0; 0 1 1; 1 0 1; 1 1 0; 1 1 1];
Marker_Size = 36;
counter = 0;
for i = 1:num_cluster
    cluster_counter = cluster(i).num_cluster;
    figure;
    subplot(ceil((cluster_counter+1)/2),2,1); 
    for j = 1: cluster(i).num_cluster
        points = cluster(i).Location(find(all(full(cluster(i).cluster_index(:,j)),2)),:);
        [center,normal,radius] = CircFit3D(points);
        radius
        gasket_radius = radiusdetection(radius,j);
        gasket_new  = Gasket(points,center,gasket_radius,normal);
        gasket = [gasket,gasket_new];
        current_color = Figure_Color(mod(j,8)+1,:);
        scatter3(points(:,1),points(:,2),points(:,3),Marker_Size,current_color);
        %Figure_Title = [Figure_Title,' radius = ',num2str(radius)];
        hold on
    end
    title(['Cluster ',num2str(i)]);
    hold off
    for j = 1: cluster(i).num_cluster
        counter = counter + 1;
        current_color = Figure_Color(mod(j,8)+1,:);
        subplot(ceil((cluster_counter+1)/2),2,j+1);
        scatter3(gasket(counter).points(:,1),...
            gasket(counter).points(:,2),...
            gasket(counter).points(:,3),...
            Marker_Size,current_color);
        title(['radius = ',num2str(gasket(counter).radius),' M',num2str(1000*gasket(counter).radius)]);
    end
end
clear Figure_Marker Figure_Color Marker_Size counter cluster_counter center ...
    normal radius gasket_radius gasket_news points current_color i j
%% Alternative Way (K-Means Clustering)
cluster = [];
for i = 1:1
    data = Location(find(all(Location(:,:,i).')),1:3,i);
    k = input('Please enter the Number of objects to look for: ');
    [cluster_index,cluster_center] = kmeans(data,k,'distance','sqEuclidean', 'start','sample');
    cluster_new = Cluster(data,cluster_index,k);
    cluster = [cluster,cluster_new];
end
clr = lines(k);
X = data;
figure, 
scatter3(X(:,1), X(:,2), X(:,3), 36, clr(cluster_index,:), 'Marker','.')
% scatter3(C(:,1), C(:,2), C(:,3), 100, clr, 'Marker','o', 'LineWidth',3)
% hold off
view(3), axis vis3d, box on, rotate3d on
xlabel('x'), ylabel('y'), zlabel('z')
clear data k cluster_new
