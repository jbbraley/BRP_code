classdef bridge < handle
%% classdef bridge
% 
% 
% 
% author: 
% create date: 15-Jul-2019 14:19:08
% classy version: 0.1.2

%% object properties
	properties
        length % span length
        girder_spacing % spacing between centers of girders
        spans % number of spans
        deck = deck(); % deck object
        girder = girder(); % girder object
	end

%% dependent properties
	properties (Dependent)
        be  % effective width
        DL_mom_mid
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = bridge()
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function be = get.be(self)
            % effective width
            A = self.girder_spacing; %sort([self.length/4, self.girder_spacing, 12*self.deck.t]);
            be = A(1);
        end
        function DL_mom_mid = get.DL_mom_mid(self)
            deck_weight = self.deck.density*self.deck.t*self.be;
            girder_weight = self.girder.density*self.girder.section_data.A;
            DL_mom_mid = (deck_weight+girder_weight)*self.length^2/8;
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
