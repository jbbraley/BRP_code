function [Delta_Min, Delta_Max, D] = GetDisplacementVector(loadType, numDOF, condK, condInd, Fixed, Load,spanlength,eleLength)
% Get Values from Parameters
%**************************************************************************
switch loadType
    case 'Truck_Forward'
        % Load
        forcesP = -1*Load.A;
        laneLoad = 0;
        
        % Spacing
        axleSpacing = round(Load.S/12);
        frontAxle = round(Load.FS/12);
        spacing = [zeros(size(axleSpacing,2),1) frontAxle*ones(size(axleSpacing,2),1) axleSpacing']; % in ft
    case 'Truck_Backward'
        % Load
        forcesP = -1*Load.A(end:-1:1);
        laneLoad = 0;
        
        % Spacing
        axleSpacing = round(Load.S/12);
        frontAxle = round(Load.FS/12);
        spacing = [zeros(size(axleSpacing,2),1) axleSpacing' frontAxle*ones(size(axleSpacing,2),1)]; % in ft
     case 'Truck_Forward_Dual'
        % Load
        forcesP = [-1*Load.A -1*Load.A];
        laneLoad = 0;
        
        dualLength = min([0.8*min(spanlength)-28, 50]); % 50 ft min spacing from back of truck 1 to front of truck 2
                                                               % or with
                                                               % the center
                                                               % axles at
                                                               % 0.4L
        % Spacing
        axleSpacing = round(Load.S/12);
        frontAxle = round(Load.FS/12);
        spacing = [zeros(size(axleSpacing,2),1) frontAxle*ones(size(axleSpacing,2),1) axleSpacing' dualLength*ones(size(axleSpacing,2),1) frontAxle*ones(size(axleSpacing,2),1) axleSpacing']; % in ft
    case 'Truck_Backward_Dual'
        % Load
        forcesP = [-1*Load.A(end:-1:1) -1*Load.A(end:-1:1)];
        laneLoad = 0;
        
        dualLength = min([0.8*min(spanlength)-28, 50]); % 50 ft min spacing from back of truck 1 to front of truck 2
                                                               % or with
                                                               % the center
                                                               % axles at
                                                               % 0.4L
        % Spacing
        axleSpacing = round(Load.S/12);
        frontAxle = round(Load.FS/12);
        spacing = [zeros(size(axleSpacing,2),1) axleSpacing' frontAxle*ones(size(axleSpacing,2),1) dualLength*ones(size(axleSpacing,2),1) axleSpacing' frontAxle*ones(size(axleSpacing,2),1)]; % in ft    
        
    case 'Tandem'
        %Load
        forcesP = -1*Load.TD;
        laneLoad = 0;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = round(Load.TS./eleLength(1));
        spacing = [0 frontAxle]; % in ft
    case 'Point'
        %Load
        % when lane loading for deflection, single point load is 25% of truck total
        forcesP = -1*Load.PM;
        laneLoad = 0;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
        spacing = 0;
    case 'Lane_PatternEven'
        %Load
        forcesP = 0;
        laneLoad = -1*Load.LaneLoad/12;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
    case 'Lane_PatternOdd'
        %Load
        forcesP = 0;
        laneLoad = -1*Load.LaneLoad/12;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
    case 'Lane_All'
        %Load
        forcesP = 0;
        laneLoad = -1*Load.LaneLoad/12;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
     case 'Dead'
         %Load
        forcesP = 0;
        laneLoad = -1;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
    case 'Distributed'
         %Load
        forcesP = 0;
        laneLoad = Load.dist_load;
        
        % Spacing
        axleSpacing = 0;
        frontAxle = 0;
end

% Construct Displacement Vector
%************************************************************************
% Point Loads
%*************************************************************************
% If loading is point load
if sum(forcesP) ~= 0    
    
    FEM = [];
    D = zeros(numDOF, numDOF/2 + max(sum(spacing,2)), size(axleSpacing,2));
    
    for j = 1:size(axleSpacing, 2) % step over rear axle spacing lengths
        for i = 1:numDOF/2 + max(sum(spacing,2)) % step truck load node by node over entire length
            
            F = zeros(numDOF,1); %initialize load vector
            
            % Incorporate nodal forces from axle load
            for k=1:length(forcesP)
                if i - sum(spacing(j,1:k)) <= numDOF/2 &&... % if axle is still on beam
                        i - sum(spacing(j,1:k)) >= 1 % if axle is still on beam
                    F(2*i-1 - 2*sum(spacing(j,1:k))) = forcesP(k); %put in applied force
                end
            end
                                
            % Remove DOFs from force vector and solve for displacement
            ft = removerows(F,Fixed);            
            
            loadsplot(j,i,:) = F(1:2:end);
    
            dt = condK\ft;
            D(condInd,i,j) = dt;
        end
    end
    
    % Find max at each DOF
    Delta_Max = max(max(D,[],3),[],2);
    Delta_Min = min(min(D,[],3),[],2);
end

% Distributed Loading
%**************************************************************************
% If loading is lane load
if any(laneLoad ~= 0)
    
    L = eleLength; % in inches
    D = zeros(numDOF,1);   
    FEM = zeros(numDOF,1); %initialize load vector
    F = [];
    
    switch loadType
        case 'Lane_All'
            % Sum distributed load as resultant shear
            FEM(3:2:end-3) = (laneLoad*L(1:end-1)+laneLoad*L(2:end))/2;
            FEM(1) =  laneLoad*L(1)/2;
            FEM(end-1) = -1*laneLoad*L(end)/2;
            
            % Sum distributed load as resultant fixed end moment
            FEM(2) = laneLoad*L(1)^2/12;
            FEM(end) =  -1*laneLoad*L(end)^2/12;             
        case 'Lane_PatternEven'
            % Add pattern load to force vector, span by span
            for n = 1:2:length(Fixed)-1
                pattern = (Fixed(n)+2):2:(Fixed(n+1)-2);
                
                % Sum distributed load as resultant shear
                FEM(pattern) = (laneLoad*L((Fixed(n)+1)/2:(Fixed(n+1)+1)/2-2)+laneLoad*L((Fixed(n)+1)/2+1:(Fixed(n+1)+1)/2-1))/2; %laneLoad*L(((Fixed(n)+1)/2+1):((Fixed(n+1)+1)/2-2));
                FEM(Fixed(n)) = laneLoad*L((Fixed(n)+1)/2)/2;
                FEM(Fixed(n+1)) = laneLoad*L((Fixed(n+1)+1)/2-1)/2;
                
                % Sum distributed load as resultant fixed end moment
                FEM(Fixed(n)+1) = laneLoad*L((Fixed(n)+1)/2)^2/12;
                FEM(Fixed(n+1)+1) = -1*laneLoad*L((Fixed(n+1)+1)/2-1)^2/12;
            end           
        case 'Lane_PatternOdd'
            % Add pattern load to force vector, span by span
            for n = 2:2:length(Fixed)-1
                pattern = Fixed(n)+2:2:Fixed(n+1)-2;
                
                FEM(pattern) = (laneLoad*L((Fixed(n)+1)/2:(Fixed(n+1)+1)/2-2)+laneLoad*L((Fixed(n)+1)/2+1:(Fixed(n+1)+1)/2-1))/2; %laneLoad*L(((Fixed(n)+1)/2+1):((Fixed(n+1)+1)/2-2));
                FEM(Fixed(n)) = laneLoad*L((Fixed(n)+1)/2)/2;
                FEM(Fixed(n+1)) = laneLoad*L((Fixed(n+1)+1)/2-1)/2;
                
                % Sum distributed load as resultant fixed end moment
                FEM(Fixed(n)+1) = laneLoad*L((Fixed(n)+1)/2)^2/12;
                FEM(Fixed(n+1)+1) = -1*laneLoad*L((Fixed(n+1)+1)/2-1)^2/12;
            end
        case 'Dead'
            % Sum distributed load as resultant shear
            FEM(3:2:end-3) = (laneLoad(1:end-1).*L(1:end-1)+laneLoad(2:end).*L(2:end))/2;
            FEM(1) = laneLoad(1)*L(1)/2;
            FEM(end-1) = laneLoad(end-1)*L(end)/2;
            
            % Sum distributed load as resultant fixed end moment
            FEM(2) = laneLoad(1)*L(1)^2/12;
            FEM(4:2:end-2) = (laneLoad(1:end-1).*L(1:end-1).^2-laneLoad(2:end).*L(2:end).^2)/12;
            FEM(end) = -1*laneLoad(end)*L(end)^2/12;
        case 'Distributed'
            % Sum distributed load as resultant shear
            FEM(3:2:end-3) = (laneLoad(1:end-1).*L(1:end-1)+laneLoad(2:end).*L(2:end))/2;
            FEM(1) = laneLoad(1)*L(1)/2;
            FEM(end-1) = laneLoad(end-1)*L(end)/2;
            
            % Sum distributed load as resultant fixed end moment
            FEM(2) = laneLoad(1)*L(1)^2/12;
            FEM(4:2:end-2) = (laneLoad(1:end-1).*L(1:end-1).^2-laneLoad(2:end).*L(2:end).^2)/12;
            FEM(end) = -1*laneLoad(end)*L(end)^2/12;
    end
    
    % Remove DOFs from force vector and solve for displacement
    ft = removerows(FEM,Fixed);
    
    dt = condK\ft; % D = K^-1*(F+FEM);
    D(condInd) = dt;
    
    % Find max at each DOF
    Delta_Max = D;
    Delta_Min = D;
end
end %function