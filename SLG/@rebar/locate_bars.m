function coords = locate_bars(self)
%% locate_bars
% 
% 
% 
% author: 
% create date: 25-Jul-2019 10:10:01
coords = zeros(self.num_bars,2);
bars_per_layer = self.num_bars/length(self.elev);
for jj = 1:length(self.elev)
    coords((jj-1)*bars_per_layer+1:jj*bars_per_layer,2) = self.elev(jj);
    coords((jj-1)*bars_per_layer+1:jj*bars_per_layer,1) = ((1:bars_per_layer)-(bars_per_layer+1)/2)*self.spacing;	
end
