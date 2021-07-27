function [fibers,init_curv] = mesh(self,d1,name1,d2,name2)
%% mesh
% discretizes mesh into rectangular elements of width, W, and height H or
% based on area
% returns a set of coordinates of the centroid of mesh elements and their respective area 
% 
% author: John Braley
% create date: 15-Jul-2019 15:07:59
	
% mesh each layer into fibers
if strcmp(name1,'dA')
    dX = sqrt(d1);
    dY = dX;
elseif strcmp(name1,'dx') || strcmp(name2,'dy')
    dX = d1;
    dY = d2;
else
    dX = d1;
    dY = dX;    
end

% mesh girder
[girder_fibers.X, girder_fibers.Y, girder_fibers.A] = self.girder.mesh(dX,'dx',dY,'dy');
girder_fibers.X = girder_fibers.X-mean([max(self.girder.shape(:,1)) min(self.girder.shape(:,1))]);

% mesh deck
[deck_fibers.X, deck_fibers.Y, deck_fibers.A] = self.deck.mesh(self.be,dX,'dx',dY,'dy');
deck_fibers.Y = deck_fibers.Y+max(self.girder.shape(:,2));
deck_fibers.X = deck_fibers.X;
		
% add deck reinforcing
rbar_coords = self.deck.rbar.locate_bars;
rb_fibers.Y = rbar_coords(:,2)+max(self.girder.shape(:,2));
rb_fibers.X = rbar_coords(:,1);
rb_fibers.A = ones(size(rb_fibers.Y))*self.deck.rbar.area;

if strcmp(self.girder.type,'PS')
    % add prestressing
    PS_coords = self.girder.locate_strands(2,2,2); % 2" bottom cover, 2" vertical spacing, 2" horizontal spacing
    PS_fibers.X = PS_coords(:,1);
    PS_fibers.Y = PS_coords(:,2);
    PS_fibers.A = ones(size(PS_fibers.Y))*self.girder.areaStrand;
    
    % add top reinforcement
    rf_coords = self.girder.locate_bars;
    rf_fibers.X = rf_coords(:,1);
    rf_fibers.Y = max(self.girder.shape(:,2))-rf_coords(:,2);
    rf_fibers.A = ones(size(rf_fibers.Y))*self.girder.rbar.area;
else
    PS_fibers.Y = []; rf_fibers.Y=[];
end
    

% figure
% plot(girder_fibers.X,girder_fibers.Y,'.');
% hold all
% plot(deck_fibers.X,deck_fibers.Y,'.');
% plot(rb_fibers.X,rb_fibers.Y,'o');
% plot(PS_fibers.X,PS_fibers.Y,'o');


% get initial strains (from dead load and prestressing)
% dead load moment
DL_mom = self.DL_mom_mid;

if strcmp(self.girder.type, 'PS')
    % prestressing axial force and moment (after losses)
    [PSforce, PSStress, ~] = self.girder.GetPSForce(self.length,self.DL_mom_mid);

    % concrete response
    init_moment = self.DL_mom_mid-PSforce*self.girder.PSecc;
    init_axial = PSforce; % tension for strands, compression for concrete
    
    % intial strain in prestressing strands
    ps_strain = ones(size(PS_fibers.Y))*PSStress/(self.girder.PSE);
    
   % convert actions to curvature 
    init_curv = init_moment/(self.girder.E*self.girder.Iut);
    init_axial_strain = init_axial/(self.girder.E*self.girder.area+self.girder.rbar.E*self.girder.rbar.total_area);
    
     % initial strain in top reinforcing bars
    rf_strain = -init_curv*(rf_fibers.Y-self.girder.section_data.yb)-init_axial_strain;
else
    init_moment = self.DL_mom_mid;
    init_axial = 0;
    
    % convert actions to curvature 
    init_curv = init_moment/(self.girder.E*self.girder.section_data.Ix);
    init_axial_strain = init_axial/(self.girder.E*self.girder.area);
    rf_strain = [];
end



% compute initial strains
% girder strain
g_strain = -init_curv*(girder_fibers.Y-self.girder.section_data.yb)-init_axial_strain;
% deck strain
d_strain = zeros(size(deck_fibers.X));
% rebar strain
rb_strain = zeros(size(rb_fibers.X));


% create array of fiber objects
total_fibers = sum([length(girder_fibers.Y) length(deck_fibers.Y) length(rb_fibers.Y) length(PS_fibers.Y) length(rf_fibers.Y)]);

for jj = 1:length(girder_fibers.Y)
    fibers{jj} = fiber();
    fibers{jj}.x = girder_fibers.X(jj);
    fibers{jj}.y = girder_fibers.Y(jj);
    fibers{jj}.area = girder_fibers.A(jj);
    fibers{jj}.material_model = self.girder.material_model;
    fibers{jj}.init_strain = g_strain(jj);
end
last_ind = length(fibers);
for kk = 1:length(deck_fibers.Y)
    fibers{kk+last_ind} = fiber();
    fibers{kk+last_ind}.x = deck_fibers.X(kk);
    fibers{kk+last_ind}.y = deck_fibers.Y(kk);
    fibers{kk+last_ind}.area = deck_fibers.A(kk);
    fibers{kk+last_ind}.material_model = self.deck.material_model;
    fibers{kk+last_ind}.init_strain = d_strain(kk);
end
last_ind = length(fibers);
for pp = 1:length(rb_fibers.Y)
    fibers{pp+last_ind} = fiber();
    fibers{pp+last_ind}.x = rb_fibers.X(pp);
    fibers{pp+last_ind}.y = rb_fibers.Y(pp);
    fibers{pp+last_ind}.area = rb_fibers.A(pp);
    fibers{pp+last_ind}.material_model = self.deck.rbar.material_model;
    fibers{pp+last_ind}.init_strain = rb_strain(pp);
end
last_ind = length(fibers);
if strcmp(self.girder.type, 'PS')
    for qq = 1:length(rf_fibers.Y)
        fibers{qq+last_ind} = fiber();
        fibers{qq+last_ind}.x = rf_fibers.X(qq);
        fibers{qq+last_ind}.y = rf_fibers.Y(qq);
        fibers{qq+last_ind}.area = rf_fibers.A(qq);
        fibers{qq+last_ind}.material_model = self.girder.rbar.material_model;
        fibers{qq+last_ind}.init_strain = rf_strain(qq);
    end
    last_ind = length(fibers);

    for rr = 1:length(PS_fibers.Y)
        fibers{rr+last_ind} = fiber();
        fibers{rr+last_ind}.x = PS_fibers.X(rr);
        fibers{rr+last_ind}.y = PS_fibers.Y(rr);
        fibers{rr+last_ind}.area = PS_fibers.A(rr);
        fibers{rr+last_ind}.material_model = self.girder.PSmat;
        fibers{rr+last_ind}.init_strain = ps_strain(rr);
    end
end

end
