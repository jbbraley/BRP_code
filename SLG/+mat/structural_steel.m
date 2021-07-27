function [sig, state] = structural_steel(eps,fy)
%% structural_steel
% 
% 
% 
% author: John Braley
% create date: 01-Aug-2019 14:48:19
if nargin<2
    fy = 36000; %psi. yield strength
end

[sig, state] = mat.steel_EP(eps,fy);
    
    
	
	
	
	
end
