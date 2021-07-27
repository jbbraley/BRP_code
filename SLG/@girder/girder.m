classdef girder < handle
%% classdef girder
% 
% 
% 
% author: 
% create date: 15-Jul-2019 14:19:37
% classy version: 0.1.2

%% object properties
	properties
        type % Rolled, Plate, RC (reinforced concrete), PS (prestressed), or Custom
        name
        density
        material_model
        section_data
        shape % coordinates of the boundary points of the girder section
        E
        fy % yield strength
	end

%% dependent properties
	properties (Dependent)
        area
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = girder(argin)
            if nargin > 0 && ~isempty(argin)
               if ischar(argin)
                   if strcmp(argin(1),'W')
                       self.E = 29000000; % psi
                       self.fy = 36000; %psi
                       self.material_model = @(eps) mat.structural_steel(eps,self.fy);
                        self.type = 'Rolled';
                    try
                        [self.section_data, self.shape] = Wsections(argin); 
                        self.name = self.section_data.Name;                        
                    catch
                    end
                   elseif strcmp(argin,'Plate')
                       self.E = 29000000; % psi
                       self.fy = 36000; %psi
                       self.material_model = @(eps) mat.structural_steel(eps,self.fy);
                        self.type = 'Plate';       
                        self.name = 'Plate'; 
                   elseif strcmp(argin(1:2),'AA') || strcmp(argin(1:2),'BT')
                       error(['Try PSgirder() instead']);
                   end
               end   
            end
		end
	end

%% ordinary methods
	methods 
	end % /ordinary

%% dependent methods
	methods 
        function area = get.area(self)
            area = polyarea([self.shape(:,1); self.shape(1,1)], [self.shape(:,2); self.shape(1,2)]);
        end
%         function shape = get.shape(self)
%             if strcmp(self.type,'PS')
%                 [self.section_data, shape] = PSsections(self.name);
%             elseif strcmp(self.type,'Rolled')
%                 
%             end
%         end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
