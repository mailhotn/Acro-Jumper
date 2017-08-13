classdef Simulation < handle & matlab.mixin.Copyable
    % This simulation integrates a system over time until
    % an event occurs, then it performs some calculations
    % and continues integrating until the next event
    
    properties
        Mod; % Model
        Con; % Controller
        IC;  % Initial Condition      
    end
    
    methods
        % Class constructor
        function sim = Simulation(varargin)
                    sim.Mod = varargin{1};
                    sim.Con = varargin{2};
        end            

        function [Xdot] = Derivative(sim,t,X)
            sim.Mod.tau = sim.Con.calc_tau(t);
            Xdot = sim.Mod.Derivative(t,X);
        end

        function [value, isterminal, direction] = Events(sim, t, X) %#ok<INUSL>
            [value, isterminal, direction] = ...
                sim.Mod.Events(X);
        end
    end
end
