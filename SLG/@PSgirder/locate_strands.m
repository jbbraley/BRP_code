function coords = locate_strands(self,BottomCover,VertSpacing,HorSpacing)
%% locate_strands
% 
% 
% 
% author: 
% create date: 24-Jul-2019 16:19:38
	
% VertSpacing = 2; % 2 inch
% HorSpacing = 2;
% BottomCover = 2; % 2 inch

for ii=1:size(self.section_data.PSrows,2)
    num_strands(ii) = sum(self.section_data.PSrows(1:ii));
end
NumRows = find((num_strands-self.numStrands)>=0,1,'first');
NumLastRow = self.numStrands-sum(self.section_data.PSrows(1:NumRows-1));

% coords = zeros(self.numStrands,2);
num_strands = [0 num_strands];
rows = self.section_data.PSrows(1:NumRows);
rows(end) = NumLastRow;

%remove missing strands
remain = rows*triu(ones(length(rows)))-self.missing_strands;
empty_rows = find(remain<=0);
rows(empty_rows) = 0;
if ~isempty(empty_rows)
    rows(empty_rows(end)+1) = remain(empty_rows(end)+1);
end

remain = [0 remain];
rows = diff(remain);
for jj = 1:(NumRows)
    coords((remain(jj)+1):remain(jj+1),2) = BottomCover+(jj-1)*VertSpacing;
    coords((remain(jj)+1):remain(jj+1),1) = ((1:rows(jj))-(rows(jj)+1)/2)*HorSpacing;
end
	
	
end
