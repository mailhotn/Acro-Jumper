function [ c, ceq ] = GA_NL_Cons( Params )
%   GA_NL_Cons Defines nonlinear constraints for the Genetic algorithm
l = 0.15;
y = 2*l*sin(Params(4)) + 2*l*sin(Params(4) + Params(5));
c = -y;
ceq = [];
end

