classdef ControllerF < handle & matlab.mixin.Copyable
    % defines the controller parameters
    
    
    properties
        amp1 = [];
%         amp2 = [];
%         EndTime = [];
        tau_max = 10;
    end
    
    methods
        % Class constructor
        function [C] = ControllerF(varargin)
%             C.EndTime = varargin{1};
            C.amp1 = varargin{1};
%             C.amp2 = varargin{3};
        end
        
        function tau = calc_tau(C, t)
            tau = C.amp1(1);
            for ii = 2:10
                tau = tau + C.amp1(ii)*sin(2*pi*(ii-1)*t);
            end
            for ii = 11:19
                tau = tau + C.amp1(ii)*cos(2*pi*(ii-10)*t);
            end
        end
        
    end
end