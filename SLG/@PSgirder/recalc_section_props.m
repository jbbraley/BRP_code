function recalc_section_props(PSobj)
%RECALC_SECTION_PROPS This function takes the geometry data of the section
%and recalculates the section properties
%   Input PS girder object
Beam = PSobj.section_data;

%% Create Cross Section
% Create Nodes
Node(1).x(1) = 0;
Node(1).y(1) = 0;

Node(1).x(2) = 0;
Node(1).y(2)= Beam.tfb(1);

Node(1).x(3) = (Beam.bfb - Beam.tw)/2;
Node(1).y(3) = sum(Beam.tfb);

Node(1).x(4) = Node(1).x(3);
Node(1).y(4) = Beam.d-sum(Beam.tft);

Node(1).x(5) = Node(1).x(4)-Beam.B4;
Node(1).y(5) = Node(1).y(4)+Beam.D4;

Node(1).x(6) = Node(1).x(5)-Beam.B5;
Node(1).y(6) = Node(1).y(5)+Beam.D3;

Node(1).x(7) = Node(1).x(6);
Node(1).y(7) = Node(1).y(6)+Beam.tft(1);

Node(2).y = Node(1).y;

Node(2).x(1) = Beam.bfb;
Node(2).x(2) = Node(2).x(1);
Node(2).x(3) = Node(1).x(3)+Beam.tw;
Node(2).x(4) = Node(2).x(3);
Node(2).x(5) = Node(2).x(4)+Beam.B4;
Node(2).x(6) = Node(2).x(5)+Beam.B5;
Node(2).x(7) = Node(1).x(7)+Beam.bft;

%concat
boundary_nodes = [Node(1).x' Node(1).y'; Node(2).x(end:-1:1)' Node(2).y(end:-1:1)'];
[ geom, iner, ~ ] = polygeom(boundary_nodes(:,1),boundary_nodes(:,2));
Beam.A = geom(1);
Beam.Ix = iner(4); % Moment of inertia
Beam.yb = geom(3); % distance from bottom to NA
Beam.yt = Beam.d-Beam.yb; % distance from top to NA
Beam.Sb = Beam.Ix/Beam.yb; % Section modulus measure from bottom flange
Beam.St = Beam.Ix/Beam.yt; % Section modulus measured from top flange
PSobj.section_data = Beam;
PSobj.shape = boundary_nodes;
end


