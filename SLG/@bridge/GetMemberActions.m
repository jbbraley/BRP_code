% (10/20/2014) NPR: Updated to reflect manual or RAMPS design

function [Moments, Shears] = GetMemberActions(self)
%*************
% Gets Member Actions and Live Load Deflection Requirement
%*************

% Initial Parameters
%*************************************************************************
% 
% % Get Truck Loads
% Parameters.Design = GetTruckLoads(Parameters.Design);

% Lengths
SpanLength = round(self.length/12)'; %in ft
TotalLength = sum(SpanLength);
E = self.girder.E; %in psi

%  Construct stiffness matrix
%*************************************************************************
numEle = TotalLength; %number of beam elements discretized 
% at 1 foot
numNode = numEle + 1; %number of joints
numDOF = numNode*2; %number of degrees of freedom (1 deflection and
% 1 slope per node)

% assemble in global stiffness matrix - assume unit/dimensionless I - in lb/ft^5
L = 12; % twelve inch lengths 
globalK = zeros(numDOF);
  
% stiffness of local element
eleK=E/L^3*[
    12, 6*L, -12, 6*L;
    6*L, 4*L^2, -6*L, 2*L^2;
    -12, -6*L, 12, -6*L;
    6*L, 2*L^2, -6*L, 4*L^2;
    ];

for i=1:numEle %for each beam element..
    Loc=2*i-1; %location of element in global matrix
    globalK(Loc:Loc+3,Loc:Loc+3) = globalK(Loc:Loc+3,Loc:Loc+3) + eleK;
end

% Set Fixed DOFS to 0 in stiffness matrix
Fixed = zeros(self.spans, 1);
for i = 1:self.spans
    Fixed(i) = sum(SpanLength(1:i));
end
Fixed = [1; 2*Fixed + 1];

condInd = 1:numDOF;
condInd = removerows(condInd', Fixed);
condK = removerows(globalK, Fixed);
condK = removerows(condK', Fixed)';

% Determine minimum stiffness for delta and span length
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get Displacement for 3 Type of Loading/3 types of responses
%**************************************************************************
loadType = {'Truck_Forward'; 'Truck_Backward'; 'Truck_Forward_Dual'; 'Truck_Backward_Dual'; 'Tandem'; 'Point'; 'Lane_PatternEven'; 'Lane_PatternOdd'; 'Lane_All'};

for i = 1:9
    if i == 5 && Tandem == 0
        continue
    end
    
    if (i == 7 || i == 8 || i == 9) && LaneLoad == 0
        continue
    end
    
    [Delta_Min(i,:), Delta_Max(i,:), D{i}] = GetDisplacementVector(loadType{i}, numDOF, condK, condInd, Fixed, Parameters);    
end

% Superimpose lane and point loading
Delta_Min(10,:) = Delta_Min(6,:) + Delta_Min(7,:); % all in in^5
Delta_Min(11,:) = Delta_Min(6,:) + Delta_Min(8,:);
Delta_Min(12,:) = Delta_Min(6,:) + Delta_Min(9,:);
Delta_Max(10,:) = Delta_Max(6,:) + Delta_Max(7,:);
Delta_Max(11,:) = Delta_Max(6,:) + Delta_Max(8,:);
Delta_Max(12,:) = Delta_Max(6,:) + Delta_Max(9,:);

% Determine Member Actions Due to Live Load Bending and Shear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Member actions are in terms of lb*in^4 and lb*in^5 for shear and moment
% Divide by each new moment of inertia when updating the beam

% Get Moment Vectors for Each Displacement Vector
%**************************************************************************
for i = 1:9
    if i == 5 && Parameters.Design.Tandem == 0
        continue
    end
    
    if (i == 7 || i == 8 || i == 9) && Parameters.Design.LaneLoad == 0
        continue
    end
    
    [M_Max(i,:), M_Min(i,:), V_Max(i,:), V_Min(i,:), ~, ~] = GetMemberForceVector(D{i}, loadType{i}, Parameters, numEle);
end

% Superimpose lane and point loading
M_Min(10,:) = M_Min(6,:) + M_Min(7,:); % all in lb.in
M_Max(11,:) = M_Max(6,:) + M_Max(7,:);
M_Min(12,:) = M_Min(6,:) + M_Min(8,:);
M_Max(10,:) = M_Max(6,:) + M_Max(8,:);
M_Min(11,:) = M_Min(6,:) + M_Min(9,:);
M_Max(12,:) = M_Max(6,:) + M_Max(9,:);

V_Min(10,:) = V_Min(6,:) + V_Min(7,:); % all in lb
V_Max(11,:) = V_Max(6,:) + V_Max(7,:);
V_Min(12,:) = V_Min(6,:) + V_Min(8,:);
V_Max(10,:) = V_Max(6,:) + V_Max(8,:);
V_Min(11,:) = V_Min(6,:) + V_Min(9,:);
V_Max(12,:) = V_Max(6,:) + V_Max(9,:);

Parameters.Design.Load.M_Max = M_Max;
Parameters.Design.Load.M_Min = M_Min;
Parameters.Design.Load.V_Max = V_Max;
Parameters.Design.Load.V_Min = V_Min;


%%
% Find max\min for each span
for i=1:length(Fixed)-1
    range = (Fixed(i)+1)/2:(Fixed(i+1)+1)/2;
    Parameters.Design.Load.maxM(i) = max(max(M_Max(:,range))); % in lb.in
    Parameters.Design.Load.minM(i) = min(min(M_Min(:,range))); 
    
    Parameters.Design.Load.maxV(i) = max(max(V_Max(:,range))); % in lb
    Parameters.Design.Load.minV(i) = min(min(V_Min(:,range)));
end

% Find max/min for points of interest
%**************************************************************************
% Simple spans: Moment at midspan and shear at supports
% Continuous spans: Positive moment at midspan for interior spans and 0.4L
% at exterior spans. Negative moment and shear over supports
maxSpan = zeros(self.spans, 2);
for i=1:self.spans
    range = (Fixed(i)+1)/2:(Fixed(i+1)+1)/2;
    % POI
    if self.spans == 1
        POI = round(0.5*(range(end)-range(1)))+range(1);
    elseif i == 1 
        POI = round(0.4*(range(end)-range(1)))+range(1);
    elseif i == self.spans 
        POI = round(0.6*(range(end)-range(1)))+range(1);
    else
        POI = round(0.5*(range(end)-range(1)))+range(1);
    end
    
    % Displacement
    maxSpan(i,1) = max(max(Delta_Max(:,Fixed(i):2:Fixed(i+1)),[],1));
    maxSpan(i,2) = max(-1*min(Delta_Min(:,Fixed(i):2:Fixed(i+1)),[],1));
    
    % Actions
    % Positive moment
    Parameters.Design.Load.maxM_POI(i,:) = max(M_Max(:,POI)); % in lb.in
    
    % Negative Moments
    Parameters.Design.Load.minM_POI(i,:) = min(M_Min(:,[(Fixed(i)+1)/2, (Fixed(i+1)+1)/2]));
    
    % Shear
    Parameters.Design.Load.maxV_POI(i,:) = max(abs(V_Max(:,[(Fixed(i)+1)/2, (Fixed(i+1)+1)/2]))); % in lb
end

% find max deflection per span length and return single value
Parameters.Design.Load.DeltaPrime = max(maxSpan,[],2);

% Determine Member Actions Due to Distributed Load on All Spans - (Dead
% Load)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Member actions are in terms of in^4 and in^5 for shear and moment
% Unit distributed load and moment of inertia assumed.  Multiply by 
% distributed load and divide by moment of inertia.

% Get Moment Vectors for Each Displacement Vector
%**************************************************************************
% Get displacement and force vector for distributed unit load
load = 'Dead';
[~, ~, DD] = GetDisplacementVector(load, numDOF, condK, condInd, Fixed, Parameters);
[M_Max(13,:), M_Min(13,:), V_Max(13,:), V_Min(13,:), ~, ~] = GetMemberForceVector(DD, load, Parameters, numEle); 

% Find max\min for each span
for i=1:length(Fixed)-1
    range = (Fixed(i)+1)/2:(Fixed(i+1)+1)/2;
    Parameters.Design.Load.maxDLM(i) = max(max(M_Max(13,range))); % in lb.in
    Parameters.Design.Load.minDLM(i) = min(min(M_Min(13,range))); 
    
    Parameters.Design.Load.maxDLV(i) = max(max(V_Max(13,range))); % in lb
    Parameters.Design.Load.minDLV(i) = min(min(V_Min(13,range)));
end

% Find max/min for points of interest
% Simple spans: Moment at midspan and shear at supports
% Continuous spans: Positive moment at midspan for interior spans and 0.4L
% at exterior spans. Negative moment and shear over supports
for i=1:self.spans
    range = (Fixed(i)+1)/2:(Fixed(i+1)+1)/2;
    
    if self.spans == 1
        POI = round(0.5*(range(end)-range(1)))+range(1);
    elseif i == 1 
        POI = round(0.4*(range(end)-range(1)))+range(1);
    elseif i == self.spans 
        POI = round(0.6*(range(end)-range(1)))+range(1);
    else
        POI = round(0.5*(range(end)-range(1)))+range(1);
    end
    
    % Positive moment
    Parameters.Design.Load.maxDLM_POI(i,1) = M_Max(13,POI); % in lb.in
    
    % Negative Moments
    Parameters.Design.Load.minDLM_POI(i,:) = M_Min(13,[(Fixed(i)+1)/2, (Fixed(i+1)+1)/2]);
    
    % Shear
    Parameters.Design.Load.maxDLV_POI(i,:) = abs(V_Max(13,[(Fixed(i)+1)/2, (Fixed(i+1)+1)/2])); % in lb
end

% Find points of contraflexure under dead load for each span
for i=1:numEle
    if (M_Max(13,i) <= 0 && M_Max(13,i+1) >= 0) || (M_Max(13,i) >= 0 && M_Max(13,i+1) <= 0)
        zeroM(i) = 1;
    else
        zeroM(i) = 0;
    end
end
zeroM(1) = 1;
zeroM(end+1) = 1;

for i=1:length(Fixed)-1
    range = (Fixed(i)+1)/2:(Fixed(i+1)+1)/2;
    Parameters.zeroMoment(i,:) = ((Fixed(i)-1)/2 + find(zeroM(range), 2)-1)*12;
    Parameters.EffectiveLength(i) = Parameters.zeroMoment(i,2) - Parameters.zeroMoment(i,1);
end
end