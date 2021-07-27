classdef deck < handle
%% classdef deck
% 
% 
% 
% author: 
% create date: 15-Jul-2019 14:18:45
% classy version: 0.1.2

%% object properties
	properties
        t   % thickness
        fc  % compressive strength
        rbar = rebar() % reinforcing bar properties
        density % density of concrete (for dead load calculations)
        material_model % function that computes stress given strain
        composite = 1  % 1 - fully composite, 0 - non-composite, or a fraction of total force that can be transfered
        delam_depth
        delam_width
        
	end

%% dependent properties
	properties (Dependent)
        E % modulus of elasticity
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = deck()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function E = get.E(self)
            E = 57000*sqrt(self.fc);
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
