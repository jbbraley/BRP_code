% Designation   -   'AASHTO' or 'BulbTee'
function [Beam, boundary_nodes] = PSsections(choice)
% load section names
shapes_file = file();
shapes_file.name = mfilename('fullpath');
shapes_file.name = 'PSshapes.csv';
shapes_contents = shapes_file.read;
format = ['%d%s%s'];  % string and floating point
for ii = 1:length(shapes_contents)
        contents = textscan(shapes_contents{ii},format,'delimiter',',');
        shape.num(ii) = contents{1};
        shape.designation(ii) = contents{2};
        shape.size(ii) = contents{3};
end

% allow user choice
if nargin==0
    disp([shape.num' convertCharsToStrings([shape.designation' shape.size'])]);
    choice = input('Choose the number of the girder shape you wish to select');
end
if ischar(choice)
    choice = find(strcmp(choice(end),shape.size));
%     if isempty(choice)
%         if type(
end
Designation = shape.designation{choice};

% if strcmp(Designation, 'AASHTO')
%     MaxSpan(:,1) = 1:6;
%     MaxSpan(:,2) = [48 70 100 120 145 167]';
% elseif strcmp(Designation, 'BT')
%     MaxSpan(:,1) = [54 63 72]';
%     MaxSpan(:,2) = [114 130 146]';
% end
    
%% Girder dimensions
AASHTO_Dims = [28 36 45 54 63 72; 4 6 7 8 5 5; 0 0 0 0 3 3; 3 3 4.5 6 4 4; 5 6 7.5 9 10 10; 5 6 7 8 8 8; 12 12 16 20 42 42; 16 18 22 26 28 28; 6 6 7 8 8 8; 276 369 560 789 1013 1085; 12.59 15.83 20.27 24.73 31.96 36.38; 22750 50980 125390 260730 521180 733320; 18 28 48 66 76 76; 3 3 4.5 6 4 4; 0 0 0 0 13 13; 5 6 7.5 9 10 10]';
BT_Dims = [54 63 72; 3.5 3.5 3.5; 2 2 2; 2 2 2; 4.5 4.5 4.5; 6 6 6; 42 42 42; 26 26 26; 6 6 6; 659 713 767; 27.63 32.12 36.60; 268077 392638 545894; 36 36 36; 2 2 2; 16 16 16; 10 10 10]';
Dims = [BT_Dims; AASHTO_Dims];
AASHTOrows = ones(6,35)*2;
AASHTOrows(:,1:9) = [6 6 4 2 2 2 2 2 2; 8 8 6 4 2 2 2 2 2; 10 10 10 8 6 4 2 2 2; 12 12 12 10 8 6 4 2 2; 12 12 12 12 10 8 6 4 2; 12 12 12 12 10 8 6 4 2];
BTrows = ones(1,35)*2;
BTrows(1:5) = [12 12 8 4 2];
rows = [repmat(BTrows,[3 1]); AASHTOrows];

Beam.d = Dims(choice,1); %Depth
Beam.bft = Dims(choice,7); %width top flange
Beam.tw = Dims(choice,9); %thickness web
Beam.tft = [Dims(choice,2) Dims(choice,3)+Dims(choice,4)]; %thickness top flange
Beam.D3 = Dims(choice,3);
Beam.D4 = Dims(choice,4);
Beam.tfb = [Dims(choice,6) Dims(choice,5)]; % thickness of bottom blange
Beam.bfb = Dims(choice,8); % width of bottom flange
Beam.B4 = Dims(choice,14);
Beam.B5 = Dims(choice,15);
Beam.A = Dims(choice,10); % total area
Beam.Ix = Dims(choice,12); % Moment of inertia
Beam.yb = Dims(choice,11); % distance from bottom to NA
Beam.yt = Beam.d-Beam.yb; % distance from top to NA
Beam.Sb = Beam.Ix/Beam.yb; % Section modulus measure from bottom flange
Beam.St = Beam.Ix/Beam.yt; % Section modulus measured from top flange
Beam.MaxStrands = Dims(choice,13); % Normal maximum number of strands used
Beam.Name = [shape.designation{choice} '-' shape.size{choice}];
Beam.PSrows = rows(choice,:);

%% Create Cross Section
% Create Nodes
Node(1).x(1) = 0;
Node(1).y(1) = 0;

Node(1).x(2) = 0;
Node(1).y(2)= Dims(choice,6);

Node(1).x(3) = Dims(choice,16);
Node(1).y(3) = sum(Beam.tfb);

Node(1).x(4) = Node(1).x(3);
Node(1).y(4) = Beam.d-sum(Beam.tft);

Node(1).x(5) = Node(1).x(4)-Dims(choice,14);
Node(1).y(5) = Node(1).y(4)+Dims(choice,4);

Node(1).x(6) = Node(1).x(5)-Dims(choice,15);
Node(1).y(6) = Node(1).y(5)+Dims(choice,3);

Node(1).x(7) = Node(1).x(6);
Node(1).y(7) = Node(1).y(6)+Dims(choice,2);

Node(2).y = Node(1).y;

Node(2).x(1) = Dims(choice,8);
Node(2).x(2) = Node(2).x(1);
Node(2).x(3) = Node(1).x(3)+Dims(choice,9);
Node(2).x(4) = Node(2).x(3);
Node(2).x(5) = Node(2).x(4)+Dims(choice,14);
Node(2).x(6) = Node(2).x(5)+Dims(choice,15);
Node(2).x(7) = Node(1).x(7)+Dims(choice,7);

%concat
boundary_nodes = [Node(1).x' Node(1).y'; Node(2).x(end:-1:1)' Node(2).y(end:-1:1)'];
end