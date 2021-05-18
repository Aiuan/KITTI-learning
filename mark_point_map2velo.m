function [pointCloud_marked]=mark_point_map2velo(map_marked, U_marked,V_marked,color)
    if sum(size(map_marked)==size(U_marked))==2 && sum(size(map_marked)==size(V_marked))==2
        mask = (map_marked~=0);
        
        x = map_marked(mask).*cosd(V_marked(mask)).*cosd(U_marked(mask));
        y = -map_marked(mask).*cosd(V_marked(mask)).*sind(U_marked(mask));
        z = map_marked(mask).*sind(V_marked(mask));
        
        hold on;
        scatter3(x,y,z,10,color,'filled');
        
        if size(x,1)==1
            pointCloud_marked = [x;y;z;ones(1,size(x,2))];
        else
            pointCloud_marked = [x';y';z';ones(1,size(x,1))];
        end
    end
end