clear all;
close all;

id = '000010';

path = './kitti/object/training/';
imgname = ['image_2/',id,'.png'];
img = imread([path, imgname]);

figure
imshow(img);
axis on;
hold on;
%% read label.txt file
labelname = ['label_2/',id,'.txt'];
fileID = fopen([path,labelname]);
label = textscan(fileID,'%s %.2f %u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f');
fclose(fileID);

%describe the type of object:
type = [label{1}];
truncated = [label{2}];
occluded = [label{3}];
alpha = [label{4}];
bbox = [label{5},label{6},label{7},label{8}];
dimensions = [label{9},label{10},label{11}];
location = [label{12},label{13},label{14}];
rotation_y = [label{15}];
%score = label{16};

%bbox
for obj = 1:size(bbox,1)
    if ~strcmp(type{obj},'DontCare')
        bbox_left = bbox(obj,1);
        bbox_top = bbox(obj,2);
        bbox_right = bbox(obj,3);
        bbox_bottom = bbox(obj,4);
        plot([bbox_left,bbox_right,bbox_right,bbox_left,bbox_left]...
            ,[bbox_top,bbox_top,bbox_bottom,bbox_bottom,bbox_top]...
            ,'r','LineWidth',2);
        text(bbox_left,bbox_top-10,type{obj},'Color','yellow');
    end  
    
end

%% read calib.txt file
calibname = ['calib/',id,'.txt'];
fileID = fopen([path,calibname]);
calib = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fileID);

temp = zeros(7,12);
for i = 2:size(calib,2)
    temp(:,i-1) = calib{i};
end

P0 = reshape(temp(1,:),4,3)';
P1 = reshape(temp(2,:),4,3)';
P2 = reshape(temp(3,:),4,3)';
P3 = reshape(temp(4,:),4,3)';
R0 = reshape(temp(5,1:9),3,3)';
Tr_velo_to_cam = reshape(temp(6,:),4,3)';
Tr_imu_to_velo = reshape(temp(7,:),4,3)';

R0_expanded = [R0,[0;0;0];[0,0,0,1]];
Tr_velo_to_cam_expanded = [Tr_velo_to_cam;[0,0,0,1]];
Tr_imu_to_velo_expanded = [Tr_imu_to_velo;[0,0,0,1]];

%% read pointcloud.bin file
cloudpointname = ['velodyne/',id,'.bin'];
fileID = fopen([path,cloudpointname]);
cloudpoint = fread(fileID,'float');
fclose(fileID);

cloudpoint = reshape(cloudpoint,[],4);

x = cloudpoint(:,1);
y = cloudpoint(:,2);
z = cloudpoint(:,3);
r = cloudpoint(:,4);
%% img+bbox




