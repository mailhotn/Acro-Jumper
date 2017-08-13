classdef Controller < handle & matlab.mixin.Copyable
    % defines the controller parameters
    
    
    properties
        amp1 = [];
        poly1 = [];
        sine1 = [];
        exponent1 = [];
        amp2 = [];
        poly2 = [];
        sine2 = [];
        exponent2 = [];
        EndTime = [];
        tau_max = 10;
    end
    
    methods
        % Class constructor
        function [C] = Controller(varargin)
            C.EndTime = varargin{1};
            C.amp1 = varargin{2};
            C.poly1 = varargin{3};
            C.sine1 = varargin{4};
            C.exponent1 = varargin{5};
            C.amp2 = varargin{6};
            C.poly2 = varargin{7};
            C.sine2 = varargin{8};
            C.exponent2 = varargin{9};
        end
        
        function tau = calc_tau(C, t)
            if t < C.EndTime(1)
                tau = C.amp1*polyval(C.poly1,t)*sin(C.sine1(1)*t+C.sine1(2))*exp(C.exponent1(1)*(t+C.exponent1(2)));
            elseif t >= C.EndTime(1)
                tau = C.amp2*polyval(C.poly2,t-C.EndTime(1))*sin(C.sine2(1)*(t-C.EndTime(1))+C.sine2(2))*exp(C.exponent2(1)*(t-C.EndTime(1)+C.exponent2(2)));
            end
                
        end
    end
end