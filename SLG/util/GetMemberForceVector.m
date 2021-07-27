function [M_Max, M_Min, V_Max, V_Min, M, V] = GetMemberForceVector(Delta, loadType, EI, numEle, EleLength,Released)

% Initialize Vectors
V = zeros(sum(numEle)+1, size(Delta,2), size(Delta,3)); 
M = V; 

q = -1;

% Basic Element stiffness matrix
eleK= @(L) 1/L^3*[12,  6*L,   -12,  6*L;
            6*L, 4*L^2, -6*L, 2*L^2;
            -12, -6*L,  12,   -6*L;
            6*L, 2*L^2, -6*L, 4*L^2];
        
% hinged element stiffness matrix (right end rotation released)
hingK = @(L) 3*1/L^3*[1, L -1 0;
                 L, L^2, -L, 0;
                 -1, -L, 1, 0;
                 0, 0, 0, 0,];
        


% Get local (internal) nodal actions from global displacement matrix
elem = 1;
for jj = 1:length(EI)
 % Check for distributed loads and populate FEM vector
    if any(strcmp(loadType, {'Lane_PatternEven'; 'Lane_PatternOdd'; 'Lane_All'; 'Dead'; 'Distributed'}))
        eleFEM = [q*EleLength(jj)/2; q*EleLength(jj)^2/12; q*EleLength(jj)/2; -1*q*EleLength(jj)^2/12]; 
    else
        eleFEM = [0;0;0;0]*ones(1,size(Delta,3));
    end    
    
    for ii = 1:numEle(jj) % for each beam element       
        Loc = 2*elem-1; % location of element in global matrix
        if any(elem==Released)
            K = hingK(EleLength(jj))*EI(jj);
        else
            K = eleK(EleLength(jj))*EI(jj);
        end     
        for kk = 1:size(Delta,2)       
            V(elem,kk,:) = K(1,:)*squeeze(Delta(Loc:Loc+3,kk,:)) - eleFEM(1,:);
            M(elem,kk,:) = -1*K(2,:)*squeeze(Delta(Loc:Loc+3,kk,:)) + eleFEM(2,:);
            V(elem+1,kk,:) = -1*K(3,:)*squeeze(Delta(Loc:Loc+3,kk,:)) + eleFEM(3,:);
            M(elem+1,kk,:) = K(4,:)*squeeze(Delta(Loc:Loc+3,kk,:)) - eleFEM(4,:);
        end
        elem = elem+1;

    end
end

% Find max at each DOF
M_Max = max(max(M,[],3),[],2);
M_Min = min(min(M,[],3),[],2);
V_Max = max(max(V,[],3),[],2);
V_Min = min(min(V,[],3),[],2);

end %function