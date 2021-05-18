%查看KITTI数据集中的数据意义
clear all;
close all;

%id
id = '000010';

% read image
path = './kitti/object/training/';
imgName = ['image_2/',id,'.png'];
img = imread([path, imgName]);
figure();
imshow(img);

% read label.txt file
labelname = ['label_2/',id,'.txt'];
fileID = fopen([path,labelname]);
label = textscan(fileID,'%s %.2f %u %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f');
fclose(fileID);
type = [label{1}];
truncated = [label{2}];
occluded = [label{3}];
alpha = [label{4}];
bbox = [label{5},label{6},label{7},label{8}];
dimensions = [label{9},label{10},label{11}];
location = [label{12},label{13},label{14}];
rotation_y = [label{15}];

% read calib.txt file
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


%read pointCloud
pointCloudname = [id,'.bin'];
fileID = fopen([path,'velodyne/',pointCloudname]);
pointCloud = fread(fileID,'float');
fclose(fileID);

pointCloud = reshape(pointCloud,4,[])';
%反射强度
r = pointCloud(:,4);
%点云坐标系中的坐标
x_in_velo = pointCloud(:,1);
y_in_velo = pointCloud(:,2);
z_in_velo = pointCloud(:,3);
pointCloud_in_velo = [x_in_velo';y_in_velo';z_in_velo';ones(1,size(pointCloud,1))];

%相机0坐标系中的坐标
pointCloud_in_camera = Tr_velo_to_cam_expanded * pointCloud_in_velo;
% x_in_camera = pointCloud_in_camera(1,:)';
% y_in_camera = pointCloud_in_camera(2,:)';
% z_in_camera = pointCloud_in_camera(3,:)';

% show_pointCloud('camera',pointCloud_in_camera);
% % 相机坐标系，画每个object的中心点和框
% write_bbox_in_camera(location,type,dimensions,rotation_y);
%% 相机坐标系，视锥体内点云
mask_front = (pointCloud_in_camera(3,:)>=0);
pointCloud_in_camera_front = pointCloud_in_camera(:,mask_front);

projection = P2 * R0_expanded * pointCloud_in_camera_front;
projection(1,:) = projection(1,:)./projection(3,:);
projection(2,:) = projection(2,:)./projection(3,:);

mask_pixel = ((projection(1,:)>=0) + (projection(1,:)<=size(img,2)) + (projection(2,:)>=0) + (projection(2,:)<=size(img,1)));
mask_pixel = (mask_pixel==4);

pointCloud_in_camera_frustum = pointCloud_in_camera_front(:,mask_pixel);
% x_in_camera_frustum = pointCloud_in_camera_frustum(1,:)';
% y_in_camera_frustum = pointCloud_in_camera_frustum(2,:)';
% z_in_camera_frustum = pointCloud_in_camera_frustum(3,:)';

show_pointCloud('camera_frustum',pointCloud_in_camera_frustum);
% 相机坐标系，画每个object的中心点和框
write_bbox_in_camera(location,type,dimensions,rotation_y);

%% 验证rotation_y和alpha
for objId = 1:size(type,1)
    if strcmp(type{objId},'DontCare')
        continue;
    end
    
    %绘制rotation_y
    direction_start_node = location(objId,:);%起始点为目标中心点x, y, z
    direction_end_node = zeros(1,3);
    direction_end_node(1,2) = direction_start_node(1,2);%在同一y值的xz平面上
    
    %沿x轴正方向的方向向量，zoom表示方向向量的长度
    zoom = 5;
    temp_direction = zoom * [1; 0];%[x; z]
    
    %目标方向转回x轴正方向所需要的转动角度,以逆时针方向为正
    %因此，从x轴正方向转到目标方向需要转动-rotation_y，以逆时针方向为正
    R = [cos(rotation_y(objId,1)), sin(rotation_y(objId,1));
        -sin(rotation_y(objId,1)), cos(rotation_y(objId,1))];
    
    after_rotate_direction = R * temp_direction;
    
    direction_end_node(1,1) = direction_start_node(1,1) + after_rotate_direction(1,1);%x
    direction_end_node(1,3) = direction_start_node(1,3) + after_rotate_direction(2,1);%z
    
    %绘制方向
    plot3(linspace(direction_start_node(1),direction_end_node(1),30),...
          linspace(direction_start_node(2),direction_end_node(2),30),...
          linspace(direction_start_node(3),direction_end_node(3),30),...
                'r','LineWidth',1);
end