% Title:        AWP Trajectory Planning and Control
% File:         prunePath.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Remove unnecessary waypoints from a path.

function [ pruned ] = prunePath( path, map_cs )

pruned = [path(1,:)];
i = 1;
while i < size(path,1)
    j = i+1;
    while true
        if (j <= size(path,1)) && freeLine(map_cs,path(i,:),path(j,:))
            j = j + 1;
        else
            i = j - 1;
            pruned = [pruned; path(i,:)];
            break;
        end
    end
end
path = pruned;

end

