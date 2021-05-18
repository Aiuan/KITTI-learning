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

%相机坐标系中的坐标
pointCloud_in_camera = Tr_velo_to_cam_expanded * pointCloud_in_velo;
% x_in_camera = pointCloud_in_camera(1,:)';
% y_in_camera = pointCloud_in_camera(2,:)';
% z_in_camera = pointCloud_in_camera(3,:)';

show_pointCloud('camera',pointCloud_in_camera);
% % 相机坐标系，画每个object的中心点和框
write_bbox_in_camera(location,type,dimensions,rotation_y);
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
% % 相机坐标系，画每个object的中心点和框
write_bbox_in_camera(location,type,dimensions,rotation_y);
%% 激光雷达坐标系，视锥体内点
mask_front = (x_in_velo>=0);
pointCloud_in_velo_front = pointCloud_in_velo(:,mask_front);

projection = P2 * R0_expanded * Tr_velo_to_cam_expanded * pointCloud_in_velo_front;
projection(1,:) = projection(1,:)./projection(3,:);
projection(2,:) = projection(2,:)./projection(3,:);

mask_pixel = ((projection(1,:)>=0) + (projection(1,:)<=size(img,2)) + (projection(2,:)>=0) + (projection(2,:)<=size(img,1)));
mask_pixel = (mask_pixel==4);

pointCloud_in_velo_frustum = pointCloud_in_velo_front(:,mask_pixel);
% x_in_velo_frustum = pointCloud_in_velo_frustum(1,:)';
% y_in_velo_frustum = pointCloud_in_velo_frustum(2,:)';
% z_in_velo_frustum = pointCloud_in_velo_frustum(3,:)';

% show_pointCloud('velo_frustum',pointCloud_in_velo_frustum);
% % 激光雷达坐标系，画每个object的中心点和框
% write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded);

%% azimuth, elevation, range建立map
pCloud = pointCloud_in_velo_frustum;
azimuth = zeros(size(pCloud,2),1);
elevation = zeros(size(pCloud,2),1);
range = zeros(size(pCloud,2),1);

for i=1:size(pCloud,2)
    azimuth(i) = round(atand(-pCloud(2,i) / pCloud(1,i)), 3);
    elevation(i) = round(atand(pCloud(3,i) / sqrt(pCloud(1:2,i)'*pCloud(1:2,i))), 2);
    range(i) = sqrt(pCloud(1:3,i)'*pCloud(1:3,i));
end

VFOV = max(elevation) - min(elevation);
HFOV = max(azimuth) - min(azimuth);

elevation_res = 0.2; %26.9/64 ≈ 0.4
[~,elevation_centers] = hist(elevation,min(elevation):elevation_res:max(elevation));
figure();
hist(elevation,min(elevation):elevation_res:max(elevation));
xlabel('elevation');
ylabel('counts');
title('elevation histogram');

azimuth_res = 0.06; %0.08
[~,azimuth_centers] = hist(azimuth,min(azimuth):azimuth_res:max(azimuth));
figure();
hist(azimuth,min(azimuth):azimuth_res:max(azimuth));
xlabel('azimuth');
ylabel('counts');
title('azimuth histogram');

for i=1:size(pCloud,2)
    [~,idx] = min(abs(azimuth_centers - azimuth(i)));
    azimuth(i) = azimuth_centers(idx);
    [~,idx] = min(abs(elevation_centers - elevation(i)));
    elevation(i) = elevation_centers(idx);
end

%u轴为azimuth_centers，v轴为elevation_centers，像素值为range
[U,V]=meshgrid(azimuth_centers,elevation_centers);

tic;
map = zeros(size(elevation_centers,2), size(azimuth_centers,2));
map_counter =  zeros(size(elevation_centers,2), size(azimuth_centers,2));
for i=1:size(map,1)
    for j=1:size(map,2)
        temp = (U(i,j)==azimuth)+(V(i,j)==elevation);
        map_counter(i,j) = sum(temp==2);
        if map_counter(i,j)~=0
            map(i,j) = min(range(temp==2));%暂取最小值
        end
    end
end
toc;

%map归一化图像显示
figure();
imshow(flipud(map / max(max(map))));
colormap(jet);
title('map');

%count up the sheltered point
map_counter_value = unique(map_counter)';
map_counter_sum = zeros(1,size(map_counter_value,2));
for i = 1:size(map_counter_value,2)
    map_counter_sum(i) = sum(sum(map_counter==map_counter_value(i)));
end
figure();
bar(map_counter_value,map_counter_sum);
xlabel('number of points in map grid');
ylabel('counts');
title('map histogram');
for i = 1:length(map_counter_value)
text(map_counter_value(i)-0.1,map_counter_sum(i)+2000,num2str(map_counter_sum(i)));
end

%% 对map进行增强
kernal = strel([0 1 0;1 1 1;0 1 0]);
map_augmented = map;

map_augmented = imdilate(map_augmented,kernal);
map_augmented = imdilate(map_augmented,kernal);
map_augmented = imdilate(map_augmented,kernal);
map_augmented = imerode(map_augmented,kernal);


%map归一化图像显示
figure();
imshow(flipud(map_augmented / max(max(map_augmented))));
colormap(jet);
title('map');
%% 利用map_augmented重建点云
map_use = map_augmented;
num_rebuild_pointCloud = sum(sum(map_use~=0));

pointCloud_in_velo_frustum_rebuild = [zeros(3,num_rebuild_pointCloud);ones(1,num_rebuild_pointCloud)];
num=0;
for i=1:size(map_use,1)
    for j=1:size(map_use,2)
        if map_use(i,j)~=0
            num=num+1;
            pointCloud_in_velo_frustum_rebuild(1,num) = map_use(i,j)*cosd(V(i,j))*cosd(U(i,j));
            pointCloud_in_velo_frustum_rebuild(2,num) = -map_use(i,j)*cosd(V(i,j))*sind(U(i,j));
            pointCloud_in_velo_frustum_rebuild(3,num) = map_use(i,j)*sind(V(i,j));
        end
    end
end

show_pointCloud('velo_frustum',pointCloud_in_velo_frustum);
% 激光雷达坐标系，画每个object的中心点和框
write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded);
title('original pointCloud');

show_pointCloud('velo_frustum',pointCloud_in_velo_frustum_rebuild);
title('rebuid pointCloud');
% 激光雷达坐标系，画每个object的中心点和框
write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded);

disp(strcat('The original pointCloud has',32,num2str(size(pointCloud_in_velo_frustum,2)),32,'points.'));
disp(strcat('The original pointCloud has',32,num2str(num_rebuild_pointCloud),32,'points.'));


%% 去除地面 直接法
tic;
map_use = map_augmented;
map_noground = zeros(size(map_use));
thred_delta = 30;

for i=1:size(map_use,2)
    %针对map的第i列操作，某个方位角上
    pointCloud_marked = mark_point_map2velo(map_use(:,i), U(:,i),V(:,i),'w');
    %某方位角，每个点xy平面的距离
    range_xy = sqrt(pointCloud_marked(1,:).^2+pointCloud_marked(2,:).^2);
    [range_xy,idx] = sort(range_xy);%按由近至远排序
    %某方位角，每个点的高
    z = pointCloud_marked(3,:);
    z = z(idx);%与range_xy排序相对应
    
    %某方位角，每个点和前一个点的 △高/△xy平面的距离
    %定义第1点的值为0，从第2点开始计算
    delta = [0,(z(2:end)-z(1:end-1))./(range_xy(2:end)-range_xy(1:end-1))];
    
    %根据delta决定非地面点mask
    %取自delta>thred_delta的点起，直至delta<0终止
    mask = (delta>thred_delta);
    mask_down = (delta<0);    
    start =0;
    for k=1:size(mask,2)
        if mask(k)
            start=1;
        else
            if mask_down(k)==1
            start=0;
            end  
            if start==1
                mask(k)=1;
            end
        end              
    end
    
    %保留的非地面点,
    mask_noground = find(map_use(:,i)~=0);
    mask_noground = mask_noground(idx(mask));
    map_noground(mask_noground,i) = map_use(mask_noground,i); 
    
end
toc

%% 利用map_noground重建点云
map_use = map_noground;
num_rebuild_pointCloud = sum(sum(map_use~=0));

pointCloud_in_velo_frustum_rebuild = [zeros(3,num_rebuild_pointCloud);ones(1,num_rebuild_pointCloud)];
num=0;
for i=1:size(map_use,1)
    for j=1:size(map_use,2)
        if map_use(i,j)~=0
            num=num+1;
            pointCloud_in_velo_frustum_rebuild(1,num) = map_use(i,j)*cosd(V(i,j))*cosd(U(i,j));
            pointCloud_in_velo_frustum_rebuild(2,num) = -map_use(i,j)*cosd(V(i,j))*sind(U(i,j));
            pointCloud_in_velo_frustum_rebuild(3,num) = map_use(i,j)*sind(V(i,j));
        end
    end
end

show_pointCloud('velo_frustum',pointCloud_in_velo_frustum_rebuild);
title('rebuid pointCloud');
% 激光雷达坐标系，画每个object的中心点和框
write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded);

disp(strcat('The original pointCloud has',32,num2str(size(pointCloud_in_velo_frustum,2)),32,'points.'));
disp(strcat('The original pointCloud has',32,num2str(num_rebuild_pointCloud),32,'points.'));

%% 对map进行增强
kernal = strel([0 1 0;1 1 1;0 1 0]);
map_augmented2 = map_noground;

map_augmented2 = imdilate(map_augmented2,kernal);
map_augmented2 = imdilate(map_augmented2,kernal);
map_augmented2 = imdilate(map_augmented2,kernal);

%map归一化图像显示
figure();
imshow(flipud(map_augmented2 / max(max(map_augmented2))));
colormap(jet);
title('map');
%% 利用map_augmented重建点云
map_use = map_augmented2;
num_rebuild_pointCloud = sum(sum(map_use~=0));

pointCloud_in_velo_frustum_rebuild = [zeros(3,num_rebuild_pointCloud);ones(1,num_rebuild_pointCloud)];
num=0;
for i=1:size(map_use,1)
    for j=1:size(map_use,2)
        if map_use(i,j)~=0
            num=num+1;
            pointCloud_in_velo_frustum_rebuild(1,num) = map_use(i,j)*cosd(V(i,j))*cosd(U(i,j));
            pointCloud_in_velo_frustum_rebuild(2,num) = -map_use(i,j)*cosd(V(i,j))*sind(U(i,j));
            pointCloud_in_velo_frustum_rebuild(3,num) = map_use(i,j)*sind(V(i,j));
        end
    end
end

show_pointCloud('velo_frustum',pointCloud_in_velo_frustum_rebuild);
title('rebuid pointCloud');
% 激光雷达坐标系，画每个object的中心点和框
write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded);

disp(strcat('The original pointCloud has',32,num2str(size(pointCloud_in_velo_frustum,2)),32,'points.'));
disp(strcat('The original pointCloud has',32,num2str(num_rebuild_pointCloud),32,'points.'));
