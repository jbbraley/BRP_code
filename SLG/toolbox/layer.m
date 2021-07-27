function [layers] = layer(shape,dy)
%% layer
% breaks cross section into a series of layers based on boundary points
% that define the shape
% 
% author: John Braley
% create date: 15-Jul-2019 15:07:59
	
% break into layers
left_coord = shape(1:length(shape)/2,:);
right_coord = flip(shape((length(shape)/2+1):end,:),1);

% break blocks into finer layers
if nargin>1
    new_coord_l = left_coord(1,:);
    new_coord_r = right_coord(1,:);
    for jj = 1:length(left_coord)-1
        [ll, rr] = delam([left_coord((jj+(0:1)),:); right_coord((jj+(0:1)),:)],dy,'dy');
        new_coord_l = vertcat(new_coord_l,ll(2:end,:));
        new_coord_r = vertcat(new_coord_r,rr(2:end,:));
    end
    right_coord = new_coord_l;
    left_coord = new_coord_r;
end

elev = unique(right_coord(:,2));
layers = zeros(4,2,length(elev)-1);
for ii = 1:(length(elev)-1)
    start_ind = find(right_coord(:,2)==elev(ii),1,'last');
    end_ind = find(right_coord(:,2)==elev(ii+1),1,'first');
    layers(:,:,ii) = [right_coord([start_ind end_ind],:); left_coord([start_ind end_ind],:)];
%     pshape{ii} = polyshape(layer([1 2 4 3],:,ii));
end	
	
end
