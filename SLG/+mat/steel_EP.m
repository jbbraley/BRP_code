function [sig, state] = steel_EP(eps,fy)
%% structural_steel
% 
% 
% 
% author: John Braley
% create date: 01-Aug-2019 14:48:19

E = 29000000; % psi

epsy = fy/E;

if eps<epsy
    sig = eps*E;
    state = 'normal';
else 
    sig = fy;
    state = 'yield';
end
    
    
	
	
	
	
end
