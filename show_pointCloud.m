function show_pointCloud(coordinate,pointCloud)
    if strcmp(coordinate,'camera')
        figure()
        scatter3(pointCloud(1,:),pointCloud(2,:),pointCloud(3,:),2,pointCloud(3,:),'filled');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        set(gca, 'color', 'k');
        axis image; 
        colormap(jet);
        colorbar;
        hold on;
        plot3(zeros(1,50),zeros(1,50),linspace(-60,60,50),'w','LineWidth',2);
        plot3(zeros(1,50),linspace(-15,3,50),zeros(1,50),'w','LineWidth',2);
        plot3(linspace(-60,60,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
        title('pointCloud in camera');
    elseif strcmp(coordinate,'camera_frustum')
        figure()
        scatter3(pointCloud(1,:),pointCloud(2,:),pointCloud(3,:),2,pointCloud(3,:),'filled');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        set(gca, 'color', 'k');
        axis image; 
        colormap(jet);
        colorbar;
        hold on;
        plot3(zeros(1,50),zeros(1,50),linspace(0,60,50),'w','LineWidth',2);
        plot3(zeros(1,50),linspace(-15,3,50),zeros(1,50),'w','LineWidth',2);
        plot3(linspace(-60,60,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
        title('pointCloud in camera frustum');
    elseif strcmp(coordinate,'velo_frustum')
        figure()
        scatter3(pointCloud(1,:),pointCloud(2,:),pointCloud(3,:),2,pointCloud(3,:),'filled');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        set(gca, 'color', 'k');
        axis image; 
        colormap(jet);
        colorbar;
        hold on;
        plot3(zeros(1,50),linspace(-15,3,50),zeros(1,50),'w','LineWidth',2);
        plot3(zeros(1,50),linspace(-60,60,50),zeros(1,50),'w','LineWidth',2);
        plot3(linspace(0,60,50),zeros(1,50),zeros(1,50),'w','LineWidth',2);
        title('pointCloud in velo frustum');
    end
end