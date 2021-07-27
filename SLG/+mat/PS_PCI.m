function [sig, state] = PS_PCI(eps,fu)
%% PS_PCI
% 
% 
% 
% author: jbb
% create date: 21-Aug-2019 17:02:36
	
if nargin<2
    fu = 270000; %psi. yield strength
end

sig = 28500000*eps;	
state = 'normal';

switch fu
    case 270000
        if eps>0.0086
            sig = fu-40./(eps-0.007);
            state = 'yield';
        end
    case 250000
        if eps>0.0076
            sig = fu-40./(eps-0.0064);
            state = 'yield'
        end
end
if eps>0.04
    sig =0;
    state = 'rupture';
end
	
	
end
