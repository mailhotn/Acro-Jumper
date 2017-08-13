function [fitness] = GetFit(AJ, Controller, X, Time, Te, Ie)
% GetFit calculates the fitness of a controller
fitness = 0;

LandEventInd = find(Ie == 4);               % the index of the landing event
if ~isempty(LandEventInd)
    LandTime = Te(LandEventInd(1));                % get the time of landing
    LandTimeInd = find(abs(Time-LandTime) < 1e-6); % find the timestep index of the land time
    Pmin = min(X(LandTimeInd:end,1));           % find the minimal coordinate of P after landing time
    d = min([Pmin, AJ.LandingQR]) - AJ.LiftOff;
    %     disp(['Distance = ' num2str(d)]);
    if d > 0.01
        fitness = fitness - 10*d;
    else
        fitness = fitness + 12 - max(X(:,3));
    end
else
    fitness = fitness + 10 - X(end,1);
end

% tau constraint
for ii = 1:length(Time)
    if abs(Controller.calc_tau(Time(ii))) > 10
        fitness = fitness + 10;
    end
end

if ~AJ.jumped
    fitness = 100;
end

end

