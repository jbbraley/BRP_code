function [new_coords_l, new_coords_r] = delam(quad_coords,val,name)
%% delam
% 
% 
% 
% author: 
% create date: 24-Jul-2019 12:35:30
height = max(quad_coords(:,2))-min(quad_coords(:,2));
sorted_coords = sortrows(quad_coords);

if height~=0
    if nargin>2 && strcmp(name,'dy')
        num_layers = ceil(height/val);
    else
        num_layers = val;
    end

    dy = height/num_layers;

    new_y = min(quad_coords(:,2)):dy:max(quad_coords(:,2));

    
    left_x = interp1(sorted_coords(1:2,2),sorted_coords(1:2,1),new_y);
    right_x = interp1(sorted_coords(3:4,2),sorted_coords(3:4,1),new_y);
% 
%     new_coords = [left_x' new_y'; (right_x') (new_y')];
    new_coords_l = [left_x' new_y'];
    new_coords_r = [(right_x') (new_y')];
else
    new_coords_l = quad_coords(1:2,:);
    new_coords_r = quad_coords(3:4,:);
end
	
	
end
