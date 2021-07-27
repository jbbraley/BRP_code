function [xcoords_out, ycoords_out, dA_out] = mesh_quad(coords,d1,name1,d2,name2)
%% mesh_quad
% 
% coords - 4x2 matrix containing x,y coordinated of the 4 corners of the
% quadrilateral
% dA - desired area of individual mesh elements
% author: John Braley
% create date: 19-Jul-2019 13:27:40

[~, yind] = sort(coords(:,2));	
[~, xind] = sort(coords(:,1));	

width = mean([abs(diff(coords(yind(1:2),1))) abs(diff(coords(yind(3:4),1)))]);
height = mean([abs(diff(coords(xind(1:2),2))) abs(diff(coords(xind(3:4),2)))]);

if strcmp(name1,'dA')
    dX = sqrt(d1);
    dY = dX;
elseif strcmp(name1,'dx') && strcmp(name2,'dy')
    dX = d1;
    dY = d2;
else
    dX = d1;
    dY = dX;    
end
x_elem = ceil(width/dX);
y_elem = ceil(height/dY);
dx = width/x_elem;
dy = height/y_elem;

coords_x_base = (1:x_elem)*dx-dx/2;
coords_y_base = (1:y_elem)*dy-dy/2;

new_coords_y = coords_y_base+mean(coords(:,2))-mean(coords_y_base);
x_offset = interp1([mean(coords(yind(1:2),2)) mean(coords(yind(3:4),2))],[mean(coords(yind(1:2),1)) mean(coords(yind(3:4),1))],new_coords_y);
new_coords_x = meshgrid(coords_x_base-mean(coords_x_base),x_offset)+meshgrid(x_offset,coords_x_base)';
 
%% reshape arrays for output
xcoords_out = reshape(new_coords_x,[],1);
ycoords_out = repmat(new_coords_y',[size(new_coords_x,2) 1]);

dA_out = dx*dy;

%% for visually checking 
% shape = polyshape(coords);
% [xc,yc] = centroid(shape);
% figure
% plot(shape)
% hold all
% plot(reshape(new_coords_x,[],1),repmat(new_coords_y',[size(new_coords_x,2) 1]),'*')

	
	
	
end
