% addpaths
addpath(genpath('C:\Users\John B\Projects_Git\BRP\2021\2B\SLG'));

structure_data = bridge();
structure_data.girder_spacing = 6.793*12; %in
structure_data.length = [140 170 140]*12; % in.
structure_data.spans = 3;
base_girder = PSgirder('AASHTO6');
base_girder.density = 150/12^3; %lb/cu.in
base_girder.fc = 6000;
base_girder.section_data.d = 7*12; % in
base_girder.recalc_section_props();
structure_data.girder = base_girder;

structure_data.deck.t = 8.5; % in.
structure_data.deck.fc = 4000; % psi
structure_data.deck.density = 150/12^3;

%% span 1
% define girder by vectors of length and section
beam_pos1 = [105.5 linspace(115, 138.25,5) 140]; %ft
beam_depth1 = [7 interp1([115 125 138.25],[7 7.9 10],linspace(115, 138.25,5),'spline') 10];

%% span 2
beam_pos2 = [linspace(141.75,162,5) 165 169 281 285 linspace(288,308.25,5) 310]; %ft
beam_depth2 = [interp1([141.75 150 162],[10 7.7 6.5],linspace(141.75, 162,5),'spline') 6.5 6.5 6.5 6.5 interp1([288 300 308.25],[6.5 7.7 10],linspace(288,308.25,5),'spline') 10];

% figure
% plot(beam_pos2,-beam_depth2)
% axis equal

%% Span 3
beam_pos3 = [linspace(311.7500, 335,5) 344.5 450]; %ft
beam_depth3 = [beam_depth1(end-1:-1:1) 7];%[interp1([311.7500 325 335],[10 7.9 7],linspace(311.7500, 344.5,5),'spline') 7 7];

beam_pos = [beam_pos1 beam_pos2 beam_pos3];
beam_depth = [beam_depth1(1) 1/2*([beam_depth1 beam_depth2 beam_depth3(1:end-1)]+...
    [beam_depth1(2:end) beam_depth2 beam_depth3])];
beam_depth([11 17]) = 7;

% get area
for ii = 1:length(beam_pos)
    beam_section(ii) = base_girder.clone; % instance of girder object containing section properties
    beam_section(ii).section_data.d = beam_depth(ii)*12;
    beam_section(ii).recalc_section_props();
    beam_area(ii) = beam_section(ii).section_data.A;
    
end
structure_data.girder = beam_section;

%% compute section properties
compGirder_props = structure_data.CompSectionProperties();
for ii =  1:length(beam_pos)
    Ix(ii) = beam_section(ii).section_data.Ix;
end
%% compute demands
% Dead Load
DL_girder = base_girder.density*beam_area; % distributed dead load % lb/in
DL_deck = structure_data.deck.density*structure_data.deck.t*structure_data.be; % lb/in

% structure_data.plot_sxn();



%% single-line model
% vertical supports
for ii = 1:structure_data.spans
    fixed_pos(ii) = sum(structure_data.length(1:ii));
end
% hinges
moment_release_pos = [140+25 140+25+120]*12;

% get beam response to self weight
[results] = GetFEApproximation(base_girder.E*Ix,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-DL_girder);

figure
for ii = 1
    plot(results.NodeX/12,[results.M_Max(ii,:)' results.M_Min(ii,:)'])
    title(load_names{ii})
    pause
end

figure
plot(results.NodeX/12,results.V_Max(1,:))

figure
for ii = 1:size(results.Delta_Max,1)
plot(results.NodeX,[results.Delta_Max(ii,1:2:end)' results.Delta_Min(ii,1:2:end)'])

end

min(results.Delta_Min(1,1:2:end))
min(results.M_Min)
max(results.M_Max)
%% validation
[sxns, sxn_inds] = unique(beam_depth);
info = ([sxns' beam_area(sxn_inds)' Ix(sxn_inds)'])
[beam_depth' diff([0 beam_pos])']



% load_names = {'Dead';'Truck_Forward'; 'Truck_Backward'; 'Truck_Forward_Dual';...
        'Truck_Backward_Dual'; 'Tandem'; 'Lane_PatternEven';'Lane_PatternOdd'; 'Lane_All'; 
        'Truck_Forward with Lane Pattern Even'; 'Truck_Forward with Lane Pattern Odd';
        'Truck_Forward with Lane All'; 'Truck_Backward with Lane Pattern Even'; 
        'Truck_Backward with Lane Pattern Odd'; 'Truck_Backward with Lane All'; 
        'Truck_Forward_Dual with Lane Pattern Even'; 'Truck_Forward_Dual with Lane Pattern Odd';
        'Truck_Forward_Dual with Lane All'; 'Truck_Backward_Dual with Lane Pattern Even'; 
        'Truck_Backward_Dual with Lane Pattern Odd';'Truck_Backward_Dual with Lane All'
        'Tandem with Lane Pattern Even'; 'Tandem with Lane Pattern Odd';'Tandem with Lane All'};

%% demands from different stages
% Dead Load (interior girder)
haunch_depth = 3; % inches
Dia_weight = [110 221]/12;
SIPs_weight = [23 45]/12;
DL_girder = base_girder.density*beam_area + Dia_weight(2); % distributed dead load % lb/in
DL_deck = haunch_depth*base_girder.section_data.bft*structure_data.deck.density+structure_data.deck.density*structure_data.deck.t*structure_data.be+SIPs_weight(2); % lb/in
SDL = 155/12; % barrier

% anchor spans position
span_loc = [165 285];

% stage 1: Erect anchor girders and place diaphragms
wDL1 = DL_girder;
wDL1(all([beam_pos>span_loc(1); beam_pos<=span_loc(2)],1))=0;
Ix1 = Ix; % all forces carried by girders only

% stage 2: Erect drop-in span girders and place diaphragms
wDL2 = DL_girder;
wDL2(any([beam_pos<=span_loc(1); beam_pos>span_loc(2)],1))=0;
Ix2 = Ix; % all forces carried by girders only

% stage 3: Place slab on anchor span for first 105 ft (on either
% side)
wDL3 = DL_deck*ones(1,length(beam_pos));
wDL3(all([beam_pos>105.5; beam_pos<=344.5],1))=0;
Ix3 = Ix; % all forces carried by girders only

% stage 4: Place slab on drop-in span (central 111 feet)
wDL4 = DL_deck*ones(1,length(beam_pos));
wDL4(any([beam_pos<=169; beam_pos>281],1))=0;
Ix4 = Ix; % anchor spans act compositely with deck
Ix4(any([beam_pos<=105.5; beam_pos>344.5],1)) = compGirder_props.Ilt(any([beam_pos<=105.5; beam_pos>344.5],1));

% stage 5: Place remainder of span
wDL5 = DL_deck*ones(1,length(beam_pos));
remain_loc = find(all([any([beam_pos<=105.5; beam_pos>169],1); any([beam_pos<=281; beam_pos>344.5],1)]));
wDL5(remain_loc) = 0;
Ix5 = Ix;
Ix5(remain_loc) = compGirder_props.Ilt(remain_loc); % composite action at all previously placed spans

% stage 6: superimposed dead-loads
wDL6 = SDL*ones(1,length(beam_pos));
Ix6 = compGirder_props.Ilt;
% stage 7: live loads
        
% compute structural responses to dead load
% get beam response to self weight
[results(1)] = GetFEApproximation(base_girder.E*Ix1,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-wDL1);
[results(2)] = GetFEApproximation(base_girder.E*Ix2,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-wDL2);
[results(3)] = GetFEApproximation(base_girder.E*Ix3,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-wDL3);
[results(4)] = GetFEApproximation(base_girder.E*Ix4,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-wDL4);
[results(5)] = GetFEApproximation(base_girder.E*Ix5,beam_pos*12,[0 fixed_pos],moment_release_pos,'Distributed',-wDL5);
[results(6)] = GetFEApproximation(base_girder.E*Ix6,beam_pos*12,[0 fixed_pos],[],'Distributed',-wDL6);

% plot dead load cases
% moment
figure
plot(results(1).NodeX/12,[results(1).M_Max(1,:)' results(2).M_Max(1,:)' results(3).M_Max(1,:)' results(4).M_Max(1,:)' results(5).M_Max(1,:)' results(6).M_Max(1,:)'])
legend

% shear
figure
plot(results(1).NodeX/12,[results(1).V_Max(1,:)' results(2).V_Max(1,:)' results(3).V_Max(1,:)' results(4).V_Max(1,:)' results(5).V_Max(1,:)' results(6).V_Max(1,:)'])
legend

% reaction forces at end of anchor spans (negative values indicate uplift
Reaction1 = [results(1).V_Max(1,1)' results(2).V_Max(1,1)' results(3).V_Max(1,1)' results(4).V_Max(1,1)' results(5).V_Max(1,1)' results(6).V_Max(1,1)'];
Reaction2 = -[results(1).V_Max(1,end)' results(2).V_Max(1,end)' results(3).V_Max(1,end)' results(4).V_Max(1,end)' results(5).V_Max(1,end)' results(6).V_Max(1,end)'];

% midspan anchor span moment
for ii = 1:length(results)
    DL_mom(:,ii) = results(ii).M_Max(1,:);
    DL_shear(:,ii) = results(ii).V_Max(1,:);
end
figure
for ii = 1:length(results)
    plot(results(1).NodeX/12,sum(DL_mom(:,1:ii),2))
    hold all
end
legend

[val, ind] = max(sum(DL_mom(1:143,1:6),2))
[val, ind] = find(results(1).NodeX==fixed_pos(1));

DL_mom([58 143 225],:)'
DL_shear([1 143],:)'

% how many times the tandem load required to cause uplift
Ix_LL = compGirder_props.Ist;
[results(7)] = GetFEApproximation(base_girder.E*Ix_LL,beam_pos*12,[0 fixed_pos],[],'Tandem',25000);
Tand_shear = results(7).V_Min(1,[1 143])
find(results(7).V{1}(1,:)==results(7).V_Min(1,1))
figure
for ii =1:size(results(7).D{1},2)
    plot(results(7).NodeX, results(7).D{1}(1:2:end,ii))
    ylim([-0.3 .2])
    drawnow
    pause(0.01)
end
figure
for ii =1:size(results(7).D{1},2)
    plot(results(7).NodeX, results(7).V{1}(:,ii))
    ylim([-50000 50000])
    drawnow
    pause(0.01)
end

figure
for ii =1:size(results(7).D{1},2)
    plot(results(7).NodeX, results(7).M{1}(:,ii))
    ylim([-6e6 1e7])
    drawnow
    pause(0.01)
end

results(7).V_Min(1,1)/141.46e3
% all LL
[results(8)] = GetFEApproximation(base_girder.E*Ix_LL,beam_pos*12,[0 fixed_pos],[],'LRFD',[]);
