function [Beam, boundary_nodes] = Plates(Parameters)
%% Wsections
% 
% 
% 
% author: 
% create date: 16-Jul-2019 13:33:28

% populate section data
Beam.d = Parameters.Beam.Int.d; %Depth
Beam.bft = Parameters.Beam.Int.bf; %width top flange
Beam.tw = Parameters.Beam.Int.tw; %thickness web
Beam.tft = Parameters.Beam.Int.tf; %thickness top flange
Beam.tfb = Parameters.Beam.Int.tf; % thickness of bottom blange
Beam.bfb = Parameters.Beam.Int.bf; % width of bottom flange
Beam.A = Parameters.Beam.Int.A; % total area
Beam.Ix = Parameters.Beam.Int.I.Ix;% Moment of inertia
Beam.yb = Parameters.Beam.Int.y.yBnc; % distance from bottom to NA
Beam.yt = Parameters.Beam.Int.y.yTnc; % distance from top to NA
Beam.Sb = Parameters.Beam.Int.S.SBnc; % Section modulus measure from bottom flange
Beam.St = Parameters.Beam.Int.S.STnc; % Section modulus measured from top flange
Beam.Name = ['Plate_' num2str(Beam.d) 'inD'];

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

