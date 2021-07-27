function [compgirder] = CompSectionProperties(self)
%Computes the section properties of the composite section

%% Composite Section Properties
% Calculate Properties of Composite Section
deck.Ast = self.deck.t*self.be*self.deck.E/self.girder(1).E;
for ii = 1:length(self.girder)
    compgirder.yBst(ii) = (deck.Ast*(self.deck.t/2 + self.girder(ii).section_data.d) + self.girder(ii).section_data.A*self.girder(ii).section_data.yb)/(deck.Ast+self.girder(ii).section_data.A);
    compgirder.yTst(ii) = (self.deck.t + self.girder(ii).section_data.d) - compgirder.yBst(ii);

    compgirder.Ist(ii) = self.girder(ii).section_data.Ix + self.girder(ii).section_data.A*(compgirder.yBst(ii)-self.girder(ii).section_data.yb)^2 +...
        deck.Ast*self.deck.t^2/12 + deck.Ast*(compgirder.yTst(ii) - self.deck.t/2)^2;

    compgirder.SDst(ii) = compgirder.Ist(ii)/compgirder.yTst(ii);
    compgirder.STst(ii) = compgirder.Ist(ii)/(compgirder.yTst(ii)-self.deck.t);
    compgirder.SBst(ii) = compgirder.Ist(ii)/compgirder.yBst(ii);
end

%% Calculate Long term Properties of Interior & Exterior Composite Section - To be used with super imposed dead loads
deck.Alt = self.deck.t*self.be/(3)*self.deck.E/self.girder(ii).E;
for ii = 1:length(self.girder)
    compgirder.yBlt(ii) = (deck.Alt*(self.deck.t/2 + self.girder(ii).section_data.d) + self.girder(ii).section_data.A*self.girder(ii).section_data.yb)/(deck.Alt+self.girder(ii).section_data.A);
    compgirder.yTlt(ii) = (self.deck.t + self.girder(ii).section_data.d) - compgirder.yBlt(ii);

    compgirder.Ilt(ii) = self.girder(ii).section_data.Ix + self.girder(ii).section_data.A*(compgirder.yBlt(ii)-self.girder(ii).section_data.yb)^2 +...
        deck.Alt*self.deck.t^2/12 + deck.Alt*(compgirder.yTlt(ii) - self.deck.t/2)^2;

    compgirder.SDlt(ii) = compgirder.Ilt(ii)/compgirder.yTlt(ii);
    compgirder.STlt(ii) = compgirder.Ilt(ii)/(compgirder.yTlt(ii)-self.deck.t);
    compgirder.SBlt(ii) = compgirder.Ilt(ii)/compgirder.yBlt(ii);
end
end

