function fh = plot_sxn(self)
%% plot_sxn
% 
% 
% 
% author: 
% create date: 05-Aug-2019 13:07:38
	
fh = figure;
girder = polyshape([self.girder.shape(:,1)-(max(self.girder.shape(:,1))+min(self.girder.shape(:,1)))/2 self.girder.shape(:,2)]);
deck_bounds = [self.be/2*[-1 -1 1 1]' self.deck.t*[0 1 1 0]'+max(self.girder.shape(:,2))];
deck = polyshape(deck_bounds);
rbar_coords = self.deck.rbar.locate_bars;
rb_fibers.Y = rbar_coords(:,2)+max(self.girder.shape(:,2));
rb_fibers.X = rbar_coords(:,1);
plot(girder);
hold all
plot(deck);
plot(rb_fibers.X, rb_fibers.Y,'o','color','red','MarkerFaceColor', 'red')
if strcmp(self.girder.type,'PS')
    if ~isempty(self.girder.rbar.num_bars) && self.girder.rbar.num_bars ~=0 
    rb_girder_coords = self.girder.locate_bars;
    rb_girder.Y = -rb_girder_coords(:,2)+max(self.girder.shape(:,2));
    rb_girder.X = rb_girder_coords(:,1);
    plot(rb_girder.X, rb_girder.Y,'o','color','red','MarkerFaceColor', 'red')
    end
    if ~isempty(self.girder.numStrands) && self.girder.numStrands ~= 0 
    PS_coords = self.girder.locate_strands(2,2,2);
    PS.Y = PS_coords(:,2);
    PS.X = PS_coords(:,1);
    plot(PS.X,PS.Y,'o','color','blue','MarkerFaceColor', 'blue')
    end
end
axis('image')
	
	
	
end
