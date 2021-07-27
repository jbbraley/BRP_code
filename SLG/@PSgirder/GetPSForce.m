function [ PSforce, PSstress, PSstress_transfer ] = GetPSForce( PSobj, span_length,DL_mom, RH )
%GETPSFORCE Calculates the actual effective prestress force after all losses
%   Based on the Girder section and the number of PS strands and their diameter
%% Stress in Prestressing Tendons
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%

%% Stress Limits
% Immediately prior to transfer
Fpi = 0.75*PSobj.PSFu; % T5.9.3-1 (psi)

% Elastic Shortening
% Midspan Moment due to member self-weight
Mg = PSobj.section_data.A*PSobj.density*span_length^2/8;
%Prestress loss due to elastic shortening in pretensioned members - S5.9.5.2.3
Fpes = (PSobj.areaStrands*Fpi*(PSobj.section_data.Ix+(PSobj.PSecc)^2*PSobj.section_data.A)-PSobj.PSecc*Mg*PSobj.section_data.A)/...
    (PSobj.areaStrands*(PSobj.section_data.Ix+PSobj.PSecc^2*PSobj.section_data.A)+PSobj.section_data.A*PSobj.section_data.Ix*PSobj.Eci/PSobj.PSE); % psi

% Prestressing stress at transfer
PSstress_transfer = Fpi-Fpes;
% Prestressing force at transfer
Pt = PSobj.areaStrands*PSstress_transfer; %lb

% Shrinkage losses
if nargin<4
    RH = 70;
end
Fpsr = (17-.15*RH)*1000; %psi S5.9.5.4.2

% Creep Losses - S5.9.5.4.3
Fcgp = Fpes/(PSobj.PSE/PSobj.Eci);
Fcdp = DL_mom*PSobj.PSecc/PSobj.section_data.Ix;
Fpcr = max(12*Fcgp-7*Fcdp,0);

%Relaxation - S5.9.5.4.4
Fpr2 = (20000-.4*Fpes-.2*(Fpsr+Fpcr))*.3;

%Total loss after transfer
DeltaFpt = max(Fpes+Fpsr+Fpcr+Fpr2);

% Final Effective prestress response
PSstress = min(.80*PSobj.PSFy, 0.75*PSobj.PSFu-DeltaFpt);

% Total Prestress Force after losses
PSforce = PSobj.areaStrands*PSstress;

end

