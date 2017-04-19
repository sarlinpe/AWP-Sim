function [ collision ] = wspaceCollision(robot, map_ws, map_cs, th1_lim, th2_lim)

collision = zeros(map_ws.GridSize(1),map_ws.GridSize(2));

for i = 1:map_ws.GridSize(1) % rows
    for j = 1:map_ws.GridSize(2) % columns
        xy = grid2world(map_ws, [i,j]);
        th = inverseKin(robot, xy);
        if all(isnan(th(:)))
            setOccupancy(map_ws, [i,j], 1, 'grid');
            collision(i,j) = 1;
            continue;
        end
        occ = 1;
        for k = 1:size(th,1)
            if th(k,1) < th1_lim(1) || th(k,1) > th1_lim(2) || ... 
                th(k,2) < th2_lim(1) || th(k,2) > th2_lim(2) || ...
                getOccupancy(map_cs, th(k,:))
                ;
            else
                occ = 0;
            end  
        end
        setOccupancy(map_ws, [i,j], occ, 'grid');
        collision(i,j) = occ;
    end
end

end

