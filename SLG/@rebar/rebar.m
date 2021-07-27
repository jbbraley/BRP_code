classdef rebar < handle
%% classdef rebar
% 
% 
% 
% author: 
% create date: 24-Jul-2019 15:45:38
% classy version: 0.1.2

%% object properties
	properties
        num_bars
        spacing % spacing in inches between centerline of bars
        elev    % distance from bottom of deck to center of rebar
        size = 0; % integer that indicates bar size
        density = 0.0868; % density of steel (for dead load calculations)
        material_model = @(eps) mat.rebar1(eps,60000); % function that computes stress given strain
        fy = 60000; % yield strength of steel
        E = 29000000 % psi modulus of elasticity
	end

%% dependent properties
	properties (Dependent)
        area  % area of single bar
        dia % diameter of a bar
        total_area
        center % center of mass of bars
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = rebar()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function area = get.area(self)
            sizes = [0 3 4 5 6 7 8 9 10 11 14 18];
            areas = [0 .11 .20 .31 .44 .60 .79 1.00 1.27 1.56 2.25 4];
            
            area = areas(sizes==self.size);    
        end
        
        function dia = get.dia(self)
            sizes = [3 4 5 6 7 8 9 10 11 14 18];
            dias = [.375 .5 .625 .75 .875 1 1.128 1.27 1.41 1.693 2.257];
            
            dia = dias(sizes==self.size);
        end
        function total_area = get.total_area(self)
            total_area = self.area*self.num_bars;
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
