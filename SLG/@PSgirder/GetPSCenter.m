function [ PSCenter ] = GetPSCenter(PSobj)
%GETPSCENTER finds centroid of prestressing strands from bottom fiber
%   
%   PSobj.numStrands  -   double: number of prestressing strands used in section
%
%   jbb     -   7/21/14
%
%   end

VertSpacing = 2; % 2 inch
BottomCover = 2; % 2 inch

for ii=1:size(PSobj.section_data.PSrows,2)
    num_strands(ii) = sum(PSobj.section_data.PSrows(1:ii));
end
NumRows = find((num_strands-PSobj.numStrands)>=0,1,'first');
NumLastRow = PSobj.numStrands-sum(PSobj.section_data.PSrows(1:NumRows-1));
PSCenter = sum([PSobj.section_data.PSrows(1:NumRows-1) NumLastRow].*([0:NumRows-1]*VertSpacing+BottomCover))/PSobj.numStrands;
end