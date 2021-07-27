classdef PSgirder < girder
%% classdef PSgirder
% 
% 
% 
% author: John Braley
% create date: 18-Jul-2019 17:00:43
% classy version: 0.1.2

%% object properties
	properties
        numStrands % number of prestressing strands
        missing_strands = 0; %number of strands that are inactive
        diaStrand % diameter of prestressing strand
        PSFu % ultimate strength of strands
        PSE % elastic modulus of strands
        fc
        PSmat % material model of strands (function returns stress from strain)
        rbar = rebar();
        rfcenter % center of reinforcing steel (from top of girder)
	end

%% dependent properties
	properties (Dependent)
        fci % initial compressive strength of concrete
%         E % elastic modulus of concrete
        Eci % elastic modulus of concrete (initial)
        PSFy % yield strength of strands
        areaStrand % area of single strand
        areaStrands % area of prestressing strands
        PScenter % centroid of prestressing strands
        PSecc % eccentricity of prestressing strands
        Iut % moment of inertia of the uncracked transformed section
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = PSgirder(argin)
            self.type = 'PS';
            self.PSFu = 270000;
            self.PSmat = @(x) mat.PS_PCI(x,self.PSFu);
            if nargin > 0 && ~isempty(argin)
                if ischar(argin)
                    try
                        [self.section_data, self.shape] = PSsections(argin);
                        self.name = self.section_data.Name;
                    catch
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
        function areaStrand = get.areaStrand(self)
            DA(:,1) = [.250 .313 .375 .438 .500 .520 .563 .600 .700];
            DA(:,2) = [.036 .058 .085 .115 .153 .167 .192 .217 .294];
            areaStrand = DA(DA(:,1)==self.diaStrand,2);
        end
        function areaStrands = get.areaStrands(self)
            areaStrands = self.numStrands*self.areaStrand;
        end
        function PScenter = get.PScenter(self)
            PScenter = GetPSCenter(self);
        end
        function PSecc = get.PSecc(self)
            PSecc = self.section_data.yb-self.PScenter;
        end
        function fci = get.fci(self)
            fci = 0.8*self.fc;
        end
        function set.fc(self,value)
            self.fc = value;
            self.E = 57000*sqrt(value);
        end
        function Eci = get.Eci(self)
            Eci = 57000*sqrt(self.fci);
        end
        function PSFy = get.PSFy(self)
            PSFy = 0.9*self.PSFu;
        end

        function Iut = get.Iut(self)
            na = (self.area*self.section_data.yt+self.rbar.total_area*self.rfcenter*self.rbar.E/self.E)/(self.area+self.rbar.total_area*self.rbar.E/self.E);
            Iut = self.section_data.Ix+self.area*(self.section_data.yt-na)^2+self.rbar.total_area*self.rbar.E/self.E*(self.rfcenter-na)^2;
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
