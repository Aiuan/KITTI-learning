clear all;
%close all;

id = '000010';

path = './kitti/object/training/';
imgname = ['image_2/',id,'.png'];
img = imread([path, imgname]);

% figure
% imshow(img);
% axis on;
% hold on;

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

% read pointcloud.bin file
pointcloudname = ['velodyne/',id,'.bin'];
fileID = fopen([path,pointcloudname]);
pointcloud = fread(fileID,'float');
fclose(fileID);

pointcloud = reshape(pointcloud,4,[])';

pcloud_index = 1:size(pointcloud,1);
x = pointcloud(:,1);
y = pointcloud(:,2);
z = pointcloud(:,3);
r = pointcloud(:,4);

% read label.txt file
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

%% depth 
mask_front = (x>=0);%只关注前方的
tmp = [x(mask_front),y(mask_front),z(mask_front),ones(size(x(mask_front),1),1)]';

projection = P2 * R0_expanded * Tr_velo_to_cam_expanded * tmp;
projection(1,:) = projection(1,:) ./  projection(3,:);
projection(2,:) = projection(2,:) ./  projection(3,:);

projection_x = projection(1,:);
projection_y = projection(2,:);
projection_depth = x(mask_front)';

% scatter(projection_x,projection_y,5,projection_depth,'filled');
% colormap(jet);
% colorbar;

%只关注image_2图片内的点
mask_in_image = ((projection(1,:)>=0) + (projection(1,:)<=size(img,2)) + (projection(2,:)>=0) + (projection(2,:)<=size(img,1)));
mask_in_image = (mask_in_image==4);

projection_img_x = projection_x(mask_in_image);
projection_img_y = projection_y(mask_in_image);
projection_img_depth = projection_depth(mask_in_image);

figure()
scatter3(projection_img_x,projection_img_y,projection_img_depth,5,projection_depth(mask_in_image),'filled');

hold on;
imshow(img);

%bbox
for obj = 1:size(bbox,1)
    if ~strcmp(type{obj},'DontCare')
        bbox_left = bbox(obj,1);
        bbox_top = bbox(obj,2);
        bbox_right = bbox(obj,3);
        bbox_bottom = bbox(obj,4);
        hold on;
        plot([bbox_left,bbox_right,bbox_right,bbox_left,bbox_left]...
            ,[bbox_top,bbox_top,bbox_bottom,bbox_bottom,bbox_top]...
            ,'w','LineWidth',2);
        text(bbox_left,bbox_top-10,[num2str(obj),type{obj}],'Color','w');
        text(bbox_left,bbox_top+10,num2str(location(obj,3)),'Color','w');
    end  
    
end

colormap(jet);
colorbar;
axis image;


%% bbox框中的depth聚类
obj = 5;%第几个obj

bbox_left = bbox(obj,1);
bbox_top = bbox(obj,2);
bbox_right = bbox(obj,3);
bbox_bottom = bbox(obj,4);
mask_in_bbox = (projection_img_x >= bbox_left)...
    + (projection_img_x <= bbox_right)...
    + (projection_img_y >= bbox_top)...
    + (projection_img_y <= bbox_bottom);
mask_in_bbox = mask_in_bbox==4;

depth_in_bbox = projection_img_depth(mask_in_bbox);
x_in_bbox = projection_img_x(mask_in_bbox);
y_in_bbox = projection_img_y(mask_in_bbox);
info_in_bbox = [x_in_bbox',y_in_bbox',depth_in_bbox'];

% dbsacn cluster
epsilon = 10;
minpts = 5;
[idx, ~] = dbscan(info_in_bbox,epsilon,minpts);

figure()
scatter3(info_in_bbox(:,1),info_in_bbox(:,2),info_in_bbox(:,3), 5, idx+1, 'filled');
grid on;
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
set(gca, 'color', 'k');
title(['epsilon = ',num2str(epsilon),', minpts = ',num2str(minpts)]);
xlabel('x');
ylabel('y');
zlabel('depth');
legend();

class_depthes = zeros(size(unique(idx),1),2);
class_depthes(:,1) = unique(idx);

for i = 1:size(unique(idx),1)
    if class_depthes(i,1) == -1
        class_depthes(i,2) = -1;
        continue;
    end
    class_i_mask = (idx == class_depthes(i,1));
    
%     figure()
%     histogram(depth_in_bbox(class_i_mask));
%     title(['class_',num2str(class_depthes(i,1))]);
    
    class_depthes(i,2) = mean(depth_in_bbox(class_i_mask));
end

disp(['object',num2str(obj),"'s depth is ",num2str(location(obj,3)),'m']);


%% 分割出每个框对应的点云
obj = 2;

%只关注bbox内的点
bbox_left = bbox(obj,1);
bbox_top = bbox(obj,2);
bbox_right = bbox(obj,3);
bbox_bottom = bbox(obj,4);
mask_in_bbox = (projection_img_x >= bbox_left)...
    + (projection_img_x <= bbox_right)...
    + (projection_img_y >= bbox_top)...
    + (projection_img_y <= bbox_bottom);
mask_in_bbox = mask_in_bbox==4;


pcloud_in_bbox = pcloud_index(mask_front);
pcloud_in_bbox = pcloud_in_bbox(mask_in_image);
pcloud_in_bbox = pcloud_in_bbox(mask_in_bbox);

%bbox 对应视锥体
figure()
%scatter3(x(pcloud_in_bbox),y(pcloud_in_bbox),z(pcloud_in_bbox),2,x(pcloud_in_bbox),'filled');
scatter3(x(pcloud_in_bbox),y(pcloud_in_bbox),z(pcloud_in_bbox),2,z(pcloud_in_bbox),'filled');
xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'color', 'k');
axis image; 
colormap(jet);
colorbar;
hold on;
plot3(zeros(1,50),zeros(1,50),linspace(-3,3,50),'w','LineWidth',2);
plot3(linspace(-3,3,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
plot3(zeros(1,50),linspace(-3,3,50),zeros(1,50),'w','LineWidth',2);
title(['In velodyne coordinate object',num2str(obj)]);


%显示bbox
if ~strcmp(type{obj},'DontCare')
    %plot the obj th center
    center = Tr_velo_to_cam_expanded \ [location(obj,:),1]';
    scatter3(center(1),center(2),center(3),30,'g','filled');

    %plot the i th bbox
    t = location(obj,:);
    l = dimensions(obj,3);
    w = dimensions(obj,2);
    h = dimensions(obj,1);
    ry = rotation_y(obj);

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
end

%% 相机坐标系中，bbox框中的点，DBSCAN聚类分析
obj = 2;

%只关注bbox内的点
bbox_left = bbox(obj,1);
bbox_top = bbox(obj,2);
bbox_right = bbox(obj,3);
bbox_bottom = bbox(obj,4);
mask_in_bbox = (projection_img_x >= bbox_left)...
    + (projection_img_x <= bbox_right)...
    + (projection_img_y >= bbox_top)...
    + (projection_img_y <= bbox_bottom);
mask_in_bbox = mask_in_bbox==4;


pcloud_in_bbox = pcloud_index(mask_front);
pcloud_in_bbox = pcloud_in_bbox(mask_in_image);
pcloud_in_bbox = pcloud_in_bbox(mask_in_bbox);

pcloud_in_camera = Tr_velo_to_cam_expanded * [x(pcloud_in_bbox), y(pcloud_in_bbox), z(pcloud_in_bbox), ones(size(x(pcloud_in_bbox),1),1)]';
x_in_camera = pcloud_in_camera(1,:)';
y_in_camera = pcloud_in_camera(2,:)';
z_in_camera = pcloud_in_camera(3,:)';

%bbox 对应视锥体
figure();
scatter3(x_in_camera,y_in_camera,z_in_camera,2,y_in_camera,'filled');
xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'color', 'k');
axis image; 
colormap(jet);
colorbar;
hold on;
plot3(zeros(1,50),zeros(1,50),linspace(-3,3,50),'w','LineWidth',2);
plot3(linspace(-3,3,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
plot3(zeros(1,50),linspace(-3,3,50),zeros(1,50),'w','LineWidth',2);
title(['In camera coordinate, object',num2str(obj)]);


%显示bbox
if ~strcmp(type{obj},'DontCare')
    %plot the obj th center
    scatter3(location(obj,1),location(obj,2),location(obj,3),30,'g','filled');

    %plot the i th bbox
    t = location(obj,:);
    l = dimensions(obj,3);
    w = dimensions(obj,2);
    h = dimensions(obj,1);
    ry = rotation_y(obj);

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
end

% 计算中心线约束
u_center_in_pixel =  (bbox_left + bbox_right)/2;
v_center_in_pixel =  (bbox_top + bbox_bottom)/2;

%假设车辆中心深度
depth_precision = 0.1;
z_center_in_camera = 0:depth_precision:max(z_in_camera);
x_center_in_camera = zeros(1, size(z_center_in_camera,2));
y_center_in_camera = zeros(1, size(z_center_in_camera,2));

Tr_cam_to_pix = P2*R0_expanded;

for i = 1:size(z_center_in_camera,2)
    temp = [u_center_in_pixel*Tr_cam_to_pix(3,1)-Tr_cam_to_pix(1,1),...
        u_center_in_pixel*Tr_cam_to_pix(3,2)-Tr_cam_to_pix(1,2);...
        v_center_in_pixel*Tr_cam_to_pix(3,1)-Tr_cam_to_pix(2,1),...
        v_center_in_pixel*Tr_cam_to_pix(3,2)-Tr_cam_to_pix(2,2)];
    temp2 = [Tr_cam_to_pix(1,4)-u_center_in_pixel*Tr_cam_to_pix(3,4)+Tr_cam_to_pix(1,3)*z_center_in_camera(i)-u_center_in_pixel*Tr_cam_to_pix(3,3)*z_center_in_camera(i);...
        Tr_cam_to_pix(2,4)-v_center_in_pixel*Tr_cam_to_pix(3,4)+Tr_cam_to_pix(2,3)*z_center_in_camera(i)-v_center_in_pixel*Tr_cam_to_pix(3,3)*z_center_in_camera(i)];
    temp3 = temp \ temp2;
    x_center_in_camera(i) = temp3(1);
    y_center_in_camera(i) = temp3(2);
end

%绘制中心约束线
hold on;
scatter3(x_center_in_camera,y_center_in_camera,z_center_in_camera,5,'w','filled');



% DBSCAN聚类
position_in_camera = [x_in_camera,y_in_camera,z_in_camera];

% dbsacn cluster
epsilon = 0.5;
minpts = 5;
[idx, corepts] = dbscan(position_in_camera,epsilon,minpts);

%dbscan cluster
clusters = unique(idx);
%绘图类颜色生成
newColororder = createColororder(size(clusters,1)-1);

% cluster result show
figure()
plot3(zeros(1,50),zeros(1,50),linspace(-3,3,50),'w','LineWidth',2,'DisplayName','camera Z');
hold on;
plot3(linspace(-3,3,50),zeros(1,50),zeros(1,50),'w','LineWidth',2,'DisplayName','camera X');
plot3(zeros(1,50),linspace(-3,3,50),zeros(1,50),'w','LineWidth',2,'DisplayName','camera Y');
grid on;
axis image;
set(gca, 'color', 'k');
title(['object ',num2str(obj),', epsilon = ',num2str(epsilon),', minpts = ',num2str(minpts)]);
xlabel('x');
ylabel('y');
zlabel('depth');

for i = 1:size(clusters,1)
    hold on;
    cluster_mask = (idx==clusters(i));
    if(clusters(i)==-1)
       scatter3(position_in_camera(cluster_mask,1),position_in_camera(cluster_mask,2),position_in_camera(cluster_mask,3), '*','r','DisplayName','Noise');
    else
       scatter3(position_in_camera(cluster_mask,1),position_in_camera(cluster_mask,2),position_in_camera(cluster_mask,3), 3, newColororder(i-1,:),'filled','DisplayName',['Class ',num2str(clusters(i))]); 
    end
    
end
%plot the obj th center
scatter3(location(obj,1),location(obj,2),location(obj,3),50,'w','p','filled','DisplayName',"object's location");

legend('TextColor','w');

% 去噪声后cluster result show
figure()
subplot(1,3,[1,2])
plot3(zeros(1,50),zeros(1,50),linspace(-3,3,50),'w','LineWidth',2,'DisplayName','camera Z');
hold on;
plot3(linspace(-3,3,50),zeros(1,50),zeros(1,50),'w','LineWidth',2,'DisplayName','camera X');
plot3(zeros(1,50),linspace(-3,3,50),zeros(1,50),'w','LineWidth',2,'DisplayName','camera Y');
grid on;
axis image;
set(gca, 'color', 'k');
title(['object ',num2str(obj),', epsilon = ',num2str(epsilon),', minpts = ',num2str(minpts)]);
xlabel('x');
ylabel('y');
zlabel('depth');
for i = 1:size(clusters,1)
    hold on;
    cluster_mask = (idx==clusters(i));
    if(clusters(i)~=-1)
       scatter3(position_in_camera(cluster_mask,1),position_in_camera(cluster_mask,2),position_in_camera(cluster_mask,3), 3, newColororder(i-1,:),'filled','DisplayName',['Class ',num2str(clusters(i))]); 
    end
    
end

%画每个object的location
scatter3(location(obj,1),location(obj,2),location(obj,3),50,'w','p','filled','DisplayName',"object's location",'MarkerEdgeColor','r');

legend('TextColor','w');

% 计算每个簇的深度
clusters_depths = zeros(size(clusters,1),1);
for i = 1:size(clusters,1)
    if (clusters(i)~=-1)
        cluster_mask = (idx==clusters(i));
        cluster_center = mean(position_in_camera(cluster_mask,:));
        clusters_depths(i) =  cluster_center(1,3);

        %画每个簇的中心
        scatter3(cluster_center(1),cluster_center(2),cluster_center(3),50,newColororder(i-1,:),'p','filled','DisplayName',['Class ',num2str(clusters(i)),' center'],'MarkerEdgeColor','r');
    end
end

%clusters depth
subplot(1,3,3);
if clusters(1)==-1
    scatter(clusters(2:end,1),clusters_depths(2:end,1),'o','filled','DisplayName',"different class's depth");
    hold on;
    plot(clusters(2:end,1),ones(size(clusters,1)-1,1)*location(obj,3),'--r','DisplayName',"object's depth");
else
    scatter(clusters,clusters_depths,'o','filled','DisplayName',"different class's depth");
    plot(clusters,ones(size(clusters,1),1)*location(obj,3),'--r','DisplayName',"object's depth");
end
grid on;
xlabel('Class');
ylabel('depth/m');
legend();
title(['object ',num2str(obj),', epsilon = ',num2str(epsilon),', minpts = ',num2str(minpts)]);

%% 计算中心线角度


%% 利用r_y修正点云
obj = 2;

%只关注bbox内的点
bbox_left = bbox(obj,1);
bbox_top = bbox(obj,2);
bbox_right = bbox(obj,3);
bbox_bottom = bbox(obj,4);
mask_in_bbox = (projection_img_x >= bbox_left)...
    + (projection_img_x <= bbox_right)...
    + (projection_img_y >= bbox_top)...
    + (projection_img_y <= bbox_bottom);
mask_in_bbox = mask_in_bbox==4;


pcloud_in_bbox = pcloud_index(mask_front);
pcloud_in_bbox = pcloud_in_bbox(mask_in_image);
pcloud_in_bbox = pcloud_in_bbox(mask_in_bbox);

pcloud_in_camera = Tr_velo_to_cam_expanded * [x(pcloud_in_bbox), y(pcloud_in_bbox), z(pcloud_in_bbox), ones(size(x(pcloud_in_bbox),1),1)]';
x_in_camera = pcloud_in_camera(1,:)';
y_in_camera = pcloud_in_camera(2,:)';
z_in_camera = pcloud_in_camera(3,:)';

%对bbox视锥体对应点云做旋转修正
R_fix = [+cos(alpha(obj)), 0, -sin(alpha(obj));
                0, 1,        0;
         sin(alpha(obj)), 0, +cos(alpha(obj))];

pcloud_fix_in_camera = R_fix * pcloud_in_camera(1:3,:);
x_fix_in_camera = pcloud_fix_in_camera(1,:)';
y_fix_in_camera = pcloud_fix_in_camera(2,:)';
z_fix_in_camera = pcloud_fix_in_camera(3,:)';

%修正后的点云图
figure();
scatter3(x_fix_in_camera,y_fix_in_camera,z_fix_in_camera,2,y_fix_in_camera,'filled');
xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'color', 'k');
axis image; 
colormap(jet);
colorbar;
hold on;
plot3(zeros(1,50),zeros(1,50),linspace(-3,3,50),'w','LineWidth',2);
plot3(linspace(-3,3,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
plot3(zeros(1,50),linspace(-3,3,50),zeros(1,50),'w','LineWidth',2);
title(['In camera coordinate, object',num2str(obj)]);

%显示bbox
if ~strcmp(type{obj},'DontCare')
    %plot the obj th center
    location_fix = R_fix * location(obj,:)';
    scatter3(location_fix(1),location_fix(2),location_fix(3),30,'g','filled');

    %plot the i th bbox
    t = location(obj,:);
    l = dimensions(obj,3);
    w = dimensions(obj,2);
    h = dimensions(obj,1);
    ry = rotation_y(obj);

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
end


% 计算中心线约束
u_center_in_pixel =  (bbox_left + bbox_right)/2;
v_center_in_pixel =  (bbox_top + bbox_bottom)/2;
%假设车辆中心深度
depth_precision = 0.1;
z_center_in_camera = 0:depth_precision:max(z_in_camera);
x_center_in_camera = zeros(1, size(z_center_in_camera,2));
y_center_in_camera = zeros(1, size(z_center_in_camera,2));

Tr_cam_to_pix = P2*R0_expanded;

for i = 1:size(z_center_in_camera,2)
    temp = [u_center_in_pixel*Tr_cam_to_pix(3,1)-Tr_cam_to_pix(1,1),...
        u_center_in_pixel*Tr_cam_to_pix(3,2)-Tr_cam_to_pix(1,2);...
        v_center_in_pixel*Tr_cam_to_pix(3,1)-Tr_cam_to_pix(2,1),...
        v_center_in_pixel*Tr_cam_to_pix(3,2)-Tr_cam_to_pix(2,2)];
    temp2 = [Tr_cam_to_pix(1,4)-u_center_in_pixel*Tr_cam_to_pix(3,4)+Tr_cam_to_pix(1,3)*z_center_in_camera(i)-u_center_in_pixel*Tr_cam_to_pix(3,3)*z_center_in_camera(i);...
        Tr_cam_to_pix(2,4)-v_center_in_pixel*Tr_cam_to_pix(3,4)+Tr_cam_to_pix(2,3)*z_center_in_camera(i)-v_center_in_pixel*Tr_cam_to_pix(3,3)*z_center_in_camera(i)];
    temp3 = temp \ temp2;
    x_center_in_camera(i) = temp3(1);
    y_center_in_camera(i) = temp3(2);
end
%绘制中心约束线
hold on;
scatter3(x_center_in_camera,y_center_in_camera,z_center_in_camera,5,'w','filled');