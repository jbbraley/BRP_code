function [X,Y,A] = mesh(self,be,d1,name1,d2,name2)
%% mesh
% discretizes mesh into rectangular elements of width, W, and height H or
% based on area
% returns a set of coordinates of the centroid of mesh elements and their respective area 
% 
% author: John Braley
% create date: 15-Jul-2019 15:07:59
	
% mesh each layer into fibers
if strcmp(name1,'dA')
    dX = sqrt(d1);
    dY = dX;
elseif strcmp(name1,'dx') || strcmp(name2,'dy')
    dX = d1;
    dY = d2;
else
    dX = d1;
    dY = dX;    
end

% create deck boundary nodes
if isempty(self.delam_depth) || self.delam_depth==0
    deck_bounds = [be/2*[-1 -1 1 1]' self.t*[0 1 1 0]'];
else
    deck_bounds = [be/2*[-1 -1 -1+delam_width/be 1-delam_width/be 1 1]' self.t*[0 1-self.delam_depth/self.t 1 1 1-self.delam_depth/self.t 0]'];
end

% break cross section into layers
layers = layer(deck_bounds,dY);

X = []; Y = []; A = [];
for ii = 1:size(layers,3)
    [xx, yy, AA] = mesh_quad(layers(:,:,ii),dX,'dx',dY,'dy');
    X = vertcat(X,xx);
    Y = vertcat(Y,yy);
    A = vertcat(A,ones(size(xx))*AA);
end
	
	
end
