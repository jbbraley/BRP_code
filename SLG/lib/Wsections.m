function [Beam, boundary_nodes] = Wsections(name)
%% Wsections
% 
% 
% 
% author: 
% create date: 16-Jul-2019 13:33:28
% load AISC shapes
shapes_file = file();
shapes_file.name = mfilename('fullpath');
shapes_file.name = 'WShapes_Current.mat';
load(shapes_file.fullname);

% Find appropriatet shape
if ischar(name)	
   shape_num = find(strcmp({WShapes.EDIStdNom},name));	
else
    disp('Please specify a valid W-shape by name')
end
	
% populate section data
Beam.d = WShapes(shape_num).d; %Depth
Beam.bft = WShapes(shape_num).bf; %width top flange
Beam.tw = WShapes(shape_num).tw; %thickness web
Beam.tft = WShapes(shape_num).tf; %thickness top flange
Beam.tfb = WShapes(shape_num).tf; % thickness of bottom blange
Beam.bfb = WShapes(shape_num).bf; % width of bottom flange
Beam.A = WShapes(shape_num).A; % total area
Beam.Ix = WShapes(shape_num).Ix;% Moment of inertia
Beam.yb = WShapes(shape_num).d/2; % distance from bottom to NA
Beam.yt = Beam.d-Beam.yb; % distance from top to NA
Beam.Sb = WShapes(shape_num).Sx; % Section modulus measure from bottom flange
Beam.St = Beam.Ix/Beam.yt; % Section modulus measured from top flange
Beam.Name = WShapes(shape_num).AISCManualLabel;

% create boundary nodes
Node(1).x(1) = 0;
Node(1).y(1) = 0;

Node(1).x(2) = 0;
Node(1).y(2)= Beam.tfb;

Node(1).x(3) = (Beam.bfb-Beam.tw)/2;
Node(1).y(3) = Beam.tfb;

Node(1).x(4) = Node(1).x(3);
Node(1).y(4) = Beam.d-sum(Beam.tft);

Node(1).x(5) = Node(1).x(4)-(Beam.bft-Beam.tw)/2;
Node(1).y(5) = Node(1).y(4);

Node(1).x(6) = Node(1).x(5);
Node(1).y(6) = Node(1).y(5)+Beam.tft;

Node(2).y = Node(1).y;

Node(2).x(1) = Beam.bfb;
Node(2).x(2) = Node(2).x(1);
Node(2).x(3) = Node(1).x(3)+Beam.tw;
Node(2).x(4) = Node(2).x(3);
Node(2).x(5) = Node(1).x(5)+Beam.bft;
Node(2).x(6) = Node(2).x(5);

%concat
boundary_nodes = [Node(1).x' Node(1).y'; Node(2).x(end:-1:1)' Node(2).y(end:-1:1)'];

