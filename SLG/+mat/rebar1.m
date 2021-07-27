function [sig, state] = rebar1(eps,fy)
%% rebar1
% 
% 
% 
% author: John Braley
% create date: 02-Aug-2019 12:12:56
	
if nargin<2
    fy = 60000; %psi. yield strength
end

[sig, state] = mat.steel_EP(eps,fy);	
	
	
	
end
