function coords = locate_bars(self)
%% locate_bars
% 
% 
% 
% author: jbb
% create date: 12-Sep-2019 12:11:37
% Rbar(:,1) = [3 4 5 6 7 8 9 10 11 14 18]; %number
% Rbar(:,2) = [.11 .20 .31 .44 .60 .79 1.00 1.27 1.56 2.25 4]; % area
% Rbar(:,3) = [.375 .5 .625 .75 .875 1 1.128 1.27 1.41 1.693 2.257]; % diameter
% 	
% barind = find(Rbar(:,1)==self.rbar.size);

if (self.rbar.num_bars-1)*(2)+4+self.rbar.dia<self.section_data.bft
    self.rfcenter = 2+self.rbar.dia/2;
    yy = self.rfcenter*ones(self.rbar.num_bars,1);
    xx = (-0.5:1/(self.rbar.num_bars-1):0.5)*(self.section_data.bft-4-self.rbar.dia);
else
    row1 = floor((self.section_data.bft-4-self.rbar.dia)/(2))+1;
    row2 = self.rbar.num_bars-row1;
    yy = 2+self.rbar.dia/2*ones(row1,1);
    yy = [yy; (4+1/2*self.rbar.dia)*ones(row2,1)];
    xx1 = (1:row1)*(2);
    xx2 = (1:row2)*(2);
    xx = [xx1-mean(xx1) xx2-mean(xx2)];
    self.rfcenter = sum(self.rbar.area*yy)/(self.rbar.total_area);
end
        
coords = [xx' yy];	
	
	
end
