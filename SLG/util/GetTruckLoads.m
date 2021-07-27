function Load = GetTruckLoads(DesignLoad)

% Truck Loads
switch DesignLoad
    case '1'
        % H 10
        Load.DesignTruckName = 'H-10';
        Load.Tandem = 0;
        
        Load.A = [4000 15000 0];
        
        Load.S = 0;
        Load.FS = 168;
        
        Load.TD = 0;
        
        Load.PM = 0;
        Load.PS = 0;
    case '2'
        % H 15
        Load.DesignTruckName = 'H-15';
        Load.Tandem = 0;
        
        Load.A = [6000 24000 0];
        
        Load.S = 0;
        Load.FS = 168;
        
        Load.LaneLoad = 480;
        
        Load.PM = 13500;
        Load.PS = 19500;
        
        Load.TD = 0;
    case '3'
        % HS 15
        Load.DesignTruckName = 'HS-15';
        Load.Tandem = 0;
        
        Load.A = [6000 24000 24000];
        
        Load.S = [168 264 360];
        Load.FS = 168;
        
        Load.LaneLoad = 480;
        
        Load.PM = 13500;
        Load.PS = 19500;
        
        Load.TD = 0;
    case '4'
        % H 20
        Load.DesignTruckName = 'H-20';
        Load.Tandem = 0;
        
        Load.A = [8000 32000 0];
        
        Load.S = 0;
        Load.FS = 168;
        
        Load.LaneLoad = 640;
        
        Load.PM = 18000;
        Load.PS = 26000;
        
        Load.TD = 0;
    case '5'
        % HS 20
        Load.DesignTruckName = 'HS-20';
        Load.Tandem = 0;
        
        Load.A = [8000 32000 32000];
        
        Load.S = [168 264 360];
        Load.FS = 168;
        Load.TS = 0;
        
        Load.LaneLoad = 640;
        
        Load.PM = 18000;
        Load.PS = 26000;
        
        Load.TD = 0;
    case '6'
        % HS 20 + Mod (HS 20 + Military Loading)
        Load.DesignTruckName = 'HS-20 + Mod';
        Load.Tandem = 1;
        
        Load.A = [8000 32000 32000];
        
        Load.S = [168 264 360];
        Load.FS = 168;
        Load.TS = 48;
        
        Load.LaneLoad = 640;
        
        Load.PM = 18000;
        Load.PS = 26000;
        
        Load.TD = 24000;
    case '9'
        % HS 25 or greater - Assume HS 25
        Load.DesignTruckName = 'HS-25';
        Load.Tandem = 1;
        
        Load.A = [10000 40000 340000];

        Load.S = [168 264 360];
        Load.FS = 168;
        Load.TS = 48;
        
        Load.LaneLoad = 640;
        
        Load.PM = 22500;
        Load.PS = 26000;
        
        Load.TD = 25000;
    case '7'
        Load.DesignTruckName = 'Pedestrian';
    case '8'
        Load.DesignTruckName = 'Railroad';
    case '0'
        Load.DesignTruckName = 'Unknown';
    case 'A'
        Load.DesignTruckName = 'HL-93'; 
        Load.Tandem = 2;
        
        Load.A = [8000 32000 32000];
        
        Load.S = [168 264 360];
        Load.FS = 168;
        Load.TS = 48;
        
        Load.LaneLoad = 640;
        
        Load.PM = 18000;
        Load.PS = 26000;
        
        Load.TD = 25000;
    case 'B'
        Load.DesignTruckName = 'Greater than HL-93';
    case 'C'
        Load.DesignTruckName = 'Other';
    otherwise
        Load.DesignTruckName = 'Not recorded';
end
end %GetTruckLoad()