function [sig, state] = conc1(eps,fc)
%% conc1
% 
% 
% 
% author: John Braley
% create date: 01-Aug-2019 13:30:02
	
if nargin<2
    fc = 5000; % psi
end

Ec = 57000*sqrt(fc);

if eps > 0
           sig = tension_envelope(eps,fc,Ec);
           state = 'cracked';
else
    [sig, state] = compression_envelope(eps,fc,Ec);
end
end

function sig = tension_envelope(eps,fc,Ec)
% no tension capacity
sig = 0;
% if eps>-0.10*fc
%     sig = Ec*sig;
% else
%     sig = 0;
% end
end

function [sig, state] = compression_envelope(eps,fc,Ec,epsu)
if nargin<4
    epsu = 0.0038;
end
state = 'normal';
% Hognestad model
fc2 = 0.85*fc;
eps0=2*fc2/Ec;
if -eps<eps0
sig = fc2*(2*-eps/eps0-(-eps/eps0)^2);
elseif -eps<epsu
    fcu = 0.85*fc2;
    sig = interp1([eps0 epsu],[fc2 fcu],-eps);
else
    sig = 0; %0.15*fc2;
    state = 'crushing';
end

sig = -sig;
end
