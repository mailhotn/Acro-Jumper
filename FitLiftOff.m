function [fitness] = FitLiftOff(AJ, X)
% FitLiftOff calculates the fitness of the liftoff phase of an acro jumper

vCOM = AJ.GetVel(X, 'CM');
fitness = -dot(vCOM,[1/sqrt(2) 1/sqrt(2)]); % dot product with 45deg from horizon unit vector
ddq = AJ.Derivative([],X);

if ddq(4) <= 0 % Check that vertical acceleration is positive
    fitness = fitness + 2;
end
if ~AJ.jumped
    fitness = 100;
end
end

