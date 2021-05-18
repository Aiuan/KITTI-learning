% 激光雷达坐标系，画每个object的中心点和框
function write_bbox_in_velo(location,type,dimensions,rotation_y,Tr_velo_to_cam_expanded)
    for i = 1:size(location,1)
        if strcmp(type{i},'DontCare')
            break;
        end

        %plot the i th center
        %center = Tr_velo_to_cam_expanded \ (R0_expanded \ [location(i,:),1]');
        center_in_velo = Tr_velo_to_cam_expanded \ [location(i,:),1]';
        scatter3(center_in_velo(1),center_in_velo(2),center_in_velo(3),30,'g','filled');

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


    end
end