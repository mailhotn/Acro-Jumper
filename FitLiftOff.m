function [fitness] = FitLiftOff(AJ, X)
% FitLiftOff calculates the fitness of the liftoff phase of an acro jumper

vCOM = AJ.GetVel(X, 'CM');
fitness = vCOM*[1/sqrt(2) 1/sqrt(2)]; % dot product with 45deg from horizon unit vector
if ~AJ.jumped
    fitness = 100;
end  


end

