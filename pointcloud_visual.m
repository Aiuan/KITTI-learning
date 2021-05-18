% 坐标系有问题

clear all;
%close all;

id = '000010';

path = './kitti/object/training/';
imgname = ['image_2/',id,'.png'];
img = imread([path, imgname]);

cloudpointname = [id,'.bin'];
fileID = fopen([path,'velodyne/',cloudpointname]);
cloudpoint = fread(fileID,'float');
fclose(fileID);

cloudpoint = reshape(cloudpoint,4,[])';

x = cloudpoint(:,1);
y = cloudpoint(:,2);
z = cloudpoint(:,3);
r = cloudpoint(:,4);

figure()
scatter3(x,y,z,2,x,'filled');
%scatter3(x,y,z,2,z,'filled');
xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'color', 'k');
axis image; 
colormap(jet);
colorbar;
hold on;
plot3(zeros(1,50),zeros(1,50),linspace(-15,3,50),'w','LineWidth',2);
plot3(linspace(-60,60,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
plot3(zeros(1,50),linspace(-60,60,50),zeros(1,50),'w','LineWidth',2);


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



%% 画每个object的中心点和框
for i = 1:size(location,1)
    if strcmp(type{i},'DontCare')
        break;
    end
    
    %plot the i th center
    %center = Tr_velo_to_cam_expanded \ (R0_expanded \ [location(i,:),1]');
    center = Tr_velo_to_cam_expanded \ [location(i,:),1]';
    scatter3(center(1),center(2),center(3),30,'g','filled');
    
    %plot the i th bbox
    t = location(i,:);
    l = dimensions(i,3);
    w = dimensions(i,2);
    h = dimensions(i,1);
    ry = rotation_y(i);

    % index for 3D bounding box faces
    face_idx = [ 1,2,6,5   % front face
                 2,3,7,6   % left face
                 3,4,8,7   % back face
                 4,1,5,8]; % right face

    % compute rotational matrix around yaw axis
    R = [+cos(ry), 0, +sin(ry);
                0, 1,        0;
         -sin(ry), 0, +cos(ry)];

    %in camera coordinate 8 corner points
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2];
    y_corners = [0,0,0,0,-h,-h,-h,-h];
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2];

    % rotate and translate 3D bounding box
    corners_3D = R*[x_corners;y_corners;z_corners];
    corners_3D(1,:) = corners_3D(1,:) + t(1);
    corners_3D(2,:) = corners_3D(2,:) + t(2);
    corners_3D(3,:) = corners_3D(3,:) + t(3);

    %转至点云坐标系
    corners_3D = Tr_velo_to_cam_expanded \ [corners_3D;ones(1,size(corners_3D,2))];

    for j = 1:size(face_idx,1)
        x_points = corners_3D(1,face_idx(j,:));
        y_points = corners_3D(2,face_idx(j,:));
        z_points = corners_3D(3,face_idx(j,:));
        plot3(linspace(x_points(1),x_points(2),10),...
            linspace(y_points(1),y_points(2),10),...
            linspace(z_points(1),z_points(2),10),...
            'w','LineWidth',2);
        plot3(linspace(x_points(2),x_points(3),10),...
            linspace(y_points(2),y_points(3),10),...
            linspace(z_points(2),z_points(3),10),...
            'w','LineWidth',2);
        plot3(linspace(x_points(3),x_points(4),10),...
            linspace(y_points(3),y_points(4),10),...
            linspace(z_points(3),z_points(4),10),...
            'w','LineWidth',2);
        plot3(linspace(x_points(4),x_points(1),10),...
            linspace(y_points(4),y_points(1),10),...
            linspace(z_points(4),z_points(1),10),...
            'w','LineWidth',2);
    end
    
    %绘制方向
    
    
end

%% 显示image_2对应视锥体内点云
mask_front = (x>=0);%只关注前方的
pointcloud_front = [x(mask_front),y(mask_front),z(mask_front),ones(size(x(mask_front),1),1)]';
x_front = pointcloud_front(1,:)';
y_front = pointcloud_front(2,:)';
z_front = pointcloud_front(3,:)';

projection = P2 * R0_expanded * Tr_velo_to_cam_expanded * pointcloud_front;
projection(1,:) = projection(1,:) ./  projection(3,:);
projection(2,:) = projection(2,:) ./  projection(3,:);

%只关注image_2对应视锥体内点云
mask_in_image = ((projection(1,:)>=0) + (projection(1,:)<=size(img,2)) + (projection(2,:)>=0) + (projection(2,:)<=size(img,1)));
mask_in_image = (mask_in_image==4);

x_in_image = x_front(mask_in_image);
y_in_image = y_front(mask_in_image);
z_in_image = z_front(mask_in_image);
pointcloud_in_image = [x_in_image; y_in_image; z_in_image];

figure()
%scatter3(x_in_image,y_in_image,z_in_image,2,z_in_image,'filled');
scatter3(x_in_image,y_in_image,z_in_image,2,x_in_image,'filled');
xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'color', 'k');
axis image; 
colormap(jet);
colorbar;
hold on;
plot3(zeros(1,50),zeros(1,50),linspace(-5,3,50),'w','LineWidth',2);
plot3(linspace(-5,60,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
plot3(zeros(1,50),linspace(-60,60,50),zeros(1,50),'w','LineWidth',2);
hold on;

%保存视锥体点云数据



