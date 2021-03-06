classdef AcroJumper < handle & matlab.mixin.Copyable
    % AcroJumper is a class that defines a two sticks jumper.

    properties
        m  = 0.3;                   % mass
        l  = 0.15;                  % length
        Ic = 0.5*0.3*(2*0.15)^2;    % moment of inertia
        g  = 10;                    % earth's gravity
        mu = 0.3;                   % coefficient of friction
        jumped = 0;                 % did it jump?
        landed = 0;                 % did it land?
        jumpedAgain = 0;            % did it jump more than once?
        LiftOff = [];               % lift off coordinates
        LandingQR = [];             % coordinates of Q/R at landing
        
        % Slip direction
        sgn_slip = [];
        
        % Phase
        Phase = 'Stick'; % Slip, Flight
        
        % Control torques
        tau = [];
        
        % Event index
        nEvents = 10;
        % 1 - Stick -> Slip
        % 2 - Slip -> Stick
        % 3 - Liftoff!
        % 4 - P collision
        % 5 - Q collision
        % 6 - R collision
        % 7 - Too much rotation
        % 8 - Q isn't below R
        % 9 - Q isn't above P
        % 10 - abs(th2)<180 
        
        % Render parameters        
        link_width=0.025;
        link_color=[0.1, 0.3, 0.8];
        RenderObj;      
        LinkRes = 10;
        LineWidth = 1;
    end
    
    
    methods
        % Class constructor
        function AJ = AcroJumper(varargin)
        AJ; %#ok
        end
               
        function Pos = GetPos(AJ, X, which)
            switch which
                case 'P'
                    Pos = [X(1), X(3)];
                case 'Q'
                    Pos = [X(1) + 2*AJ.l*cos(X(5)), X(3) + 2*AJ.l*sin(X(5))];
                case 'R'
                    Pos = [X(1) + 2*AJ.l*cos(X(5)), X(3) + 2*AJ.l*sin(X(5))] +...
                        [2*AJ.l*cos(X(5) + X(7)), 2*AJ.l*sin(X(5) + X(7))];
                case 'CM1'
                    Pos = [X(1) + AJ.l*cos(X(5)), X(3) + AJ.l*sin(X(5))];
                case 'CM2'
                    Pos = [X(1) + 2*AJ.l*cos(X(5)), X(3) + 2*AJ.l*sin(X(5))] +...
                        [AJ.l*cos(X(5) + X(7)), AJ.l*sin(X(5) + X(7))];
                case 'CM'
                    CM2 = [X(1) + 2*AJ.l*cos(X(5)), X(3) + 2*AJ.l*sin(X(5))] +...
                        [AJ.l*cos(X(5) + X(7)), AJ.l*sin(X(5) + X(7))];
                    CM1 = [X(1) + AJ.l*cos(X(5)), X(3) + AJ.l*sin(X(5))];
                    Pos = (CM1 + CM2)/2;
                otherwise
                    error('No such position');
            end
        end
        
        function [xdot, ydot] = GetVel(AJ, X, which) %#ok
            switch which
                case 'P'
                    xdot = X(2,:);
                    ydot = X(4,:);
                otherwise
                    error('No such position');
            end
        end        
        
        function [xdotdot, ydotdot] = GetAcc(AJ, X, which)
        switch which
            case 'COM'
                Xdot = AJ.Derivative([],X);
                x = X(1); dx = X(2); y = X(3); dy = X(4); th1 = X(5); dth1 = X(6); %#ok
                th2 = X(7); dth2 = X(8); ddx = Xdot(2); ddy = Xdot(4); ddth1 = Xdot(6); ddth2 = Xdot(8);
                xdotdot = ddx - (3*ddth1*AJ.l*sin(th1))/2 - (dth1^2*AJ.l*cos(th1 + th2))/2 - (dth2^2*AJ.l*cos(th1 + th2))/2 - (3*dth1^2*AJ.l*cos(th1))/2 - (ddth1*AJ.l*sin(th1 + th2))/2 - (ddth2*AJ.l*sin(th1 + th2))/2 - dth1*dth2*AJ.l*cos(th1 + th2);
                ydotdot = ddy + (3*ddth1*AJ.l*cos(th1))/2 - (dth1^2*AJ.l*sin(th1 + th2))/2 - (dth2^2*AJ.l*sin(th1 + th2))/2 - (3*dth1^2*AJ.l*sin(th1))/2 + (ddth1*AJ.l*cos(th1 + th2))/2 + (ddth2*AJ.l*cos(th1 + th2))/2 - dth1*dth2*AJ.l*sin(th1 + th2);
        end
        end
        
        function [Ft, Fn] = GetReactionForces(AJ, X)
        x = X(1); dx = X(2); y = X(3); dy = X(4); th1 = X(5); dth1 = X(6); %#ok
        th2 = X(7); dth2 = X(8);
        switch AJ.Phase
            case 'Stick'
                Fn = (27*AJ.tau*cos(th1) - 21*AJ.tau*cos(th1 + th2) + 27*AJ.tau*cos(th1 - th2) - 9*AJ.tau*cos(th1 + 2*th2) - 41*AJ.g*AJ.l*AJ.m - 3*AJ.g*AJ.l*AJ.m*cos(2*th1 + 2*th2) + 10*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2) + 10*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2) + 90*dth1^2*AJ.l^2*AJ.m*sin(th1) + 18*dth1^2*AJ.l^2*AJ.m*sin(th1 - th2) - 6*dth1^2*AJ.l^2*AJ.m*sin(th1 + 2*th2) + 18*dth2^2*AJ.l^2*AJ.m*sin(th1 - th2) + 27*AJ.g*AJ.l*AJ.m*cos(2*th1) + 9*AJ.g*AJ.l*AJ.m*cos(2*th2) + 20*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2) + 36*dth1*dth2*AJ.l^2*AJ.m*sin(th1 - th2))/(2*AJ.l*(9*cos(2*th2) - 23));
                Ft = (21*AJ.tau*sin(th1 + th2) - 27*AJ.tau*sin(th1) - 27*AJ.tau*sin(th1 - th2) + 9*AJ.tau*sin(th1 + 2*th2) + 10*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2) + 10*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2) + 3*AJ.g*AJ.l*AJ.m*sin(2*th1 + 2*th2) + 90*dth1^2*AJ.l^2*AJ.m*cos(th1) + 18*dth1^2*AJ.l^2*AJ.m*cos(th1 - th2) - 6*dth1^2*AJ.l^2*AJ.m*cos(th1 + 2*th2) + 18*dth2^2*AJ.l^2*AJ.m*cos(th1 - th2) - 27*AJ.g*AJ.l*AJ.m*sin(2*th1) + 20*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2) + 36*dth1*dth2*AJ.l^2*AJ.m*cos(th1 - th2))/(2*AJ.l*(9*cos(2*th2) - 23));
            case 'Slip'
                Fn = (3*AJ.tau*cos(th1 + th2) - 81*AJ.tau*cos(th1) - 27*AJ.tau*cos(th1 - th2) + 9*AJ.tau*cos(th1 + 2*th2) + 41*AJ.g*AJ.l*AJ.m + 2*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2) + 2*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2) - 54*dth1^2*AJ.l^2*AJ.m*sin(th1) - 18*dth1^2*AJ.l^2*AJ.m*sin(th1 - th2) + 6*dth1^2*AJ.l^2*AJ.m*sin(th1 + 2*th2) - 18*dth2^2*AJ.l^2*AJ.m*sin(th1 - th2) - 9*AJ.g*AJ.l*AJ.m*cos(2*th2) + 4*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2) - 36*dth1*dth2*AJ.l^2*AJ.m*sin(th1 - th2))/(2*AJ.l*(27*cos(2*th1) - 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) + 27*AJ.mu*AJ.sgn_slip*sin(2*th1) - 3*AJ.mu*AJ.sgn_slip*sin(2*th1 + 2*th2) + 41));
                Ft = -AJ.mu*AJ.sgn_slip*Fn;
            case 'Flight' 
                Fn = NaN;
                Ft = NaN;
        end
        end
        
        function [Xdot] = Derivative(AJ, t, X) %#ok<INUSL>
        x = X(1); dx = X(2); y = X(3); dy = X(4); th1 = X(5); dth1 = X(6); %#ok
        th2 = X(7); dth2 = X(8);
        switch AJ.Phase
            case 'Stick'
                Xdot = [X(2);
                    0;
                    X(4);
                    0;
                    X(6);
                    -(3*(4*dth1^2*AJ.l^2*AJ.m*sin(th2) - 3*AJ.tau*cos(th2) - 2*AJ.tau + 4*dth2^2*AJ.l^2*AJ.m*sin(th2) - (9*AJ.g*AJ.l*AJ.m*cos(th1))/2 + 3*dth1^2*AJ.l^2*AJ.m*sin(2*th2) + (3*AJ.g*AJ.l*AJ.m*cos(th1 + 2*th2))/2 + 8*dth1*dth2*AJ.l^2*AJ.m*sin(th2)))/(2*AJ.l^2*AJ.m*(9*cos(th2)^2 - 16));
                    X(8);
                    (3*(20*dth1^2*AJ.l^2*AJ.m*sin(th2) - 6*AJ.tau*cos(th2) - 10*AJ.tau + 4*dth2^2*AJ.l^2*AJ.m*sin(th2) + (7*AJ.g*AJ.l*AJ.m*cos(th1 + th2))/2 - (9*AJ.g*AJ.l*AJ.m*cos(th1))/2 + 6*dth1^2*AJ.l^2*AJ.m*sin(2*th2) + 3*dth2^2*AJ.l^2*AJ.m*sin(2*th2) - (9*AJ.g*AJ.l*AJ.m*cos(th1 - th2))/2 + (3*AJ.g*AJ.l*AJ.m*cos(th1 + 2*th2))/2 + 8*dth1*dth2*AJ.l^2*AJ.m*sin(th2) + 6*dth1*dth2*AJ.l^2*AJ.m*sin(2*th2)))/(2*AJ.l^2*AJ.m*(9*cos(th2)^2 - 16))];
            case 'Slip'
                Xdot = [X(2);
                    (21*AJ.tau*sin(th1 + th2) - 27*AJ.tau*sin(th1) - 27*AJ.tau*sin(th1 - th2) + 9*AJ.tau*sin(th1 + 2*th2) + 10*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2) + 10*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2) + 3*AJ.g*AJ.l*AJ.m*sin(2*th1 + 2*th2) + 90*dth1^2*AJ.l^2*AJ.m*cos(th1) - 21*AJ.mu*AJ.sgn_slip*AJ.tau*cos(th1 + th2) + 18*dth1^2*AJ.l^2*AJ.m*cos(th1 - th2) - 6*dth1^2*AJ.l^2*AJ.m*cos(th1 + 2*th2) + 18*dth2^2*AJ.l^2*AJ.m*cos(th1 - th2) + 27*AJ.mu*AJ.sgn_slip*AJ.tau*cos(th1) + 27*AJ.mu*AJ.sgn_slip*AJ.tau*cos(th1 - th2) - 9*AJ.mu*AJ.sgn_slip*AJ.tau*cos(th1 + 2*th2) - 27*AJ.g*AJ.l*AJ.m*sin(2*th1) + 20*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2) + 36*dth1*dth2*AJ.l^2*AJ.m*cos(th1 - th2) - 41*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip + 90*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1) + 18*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 - th2) - 6*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + 2*th2) + 18*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 - th2) + 27*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1) + 9*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th2) - 3*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + 2*th2) + 10*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + th2) + 10*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + th2) + 36*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 - th2) + 20*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + th2))/(AJ.l*AJ.m*(27*cos(2*th1) - 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) + 27*AJ.mu*AJ.sgn_slip*sin(2*th1) - 3*AJ.mu*AJ.sgn_slip*sin(2*th1 + 2*th2) + 41));
                    X(4);
                    0;
                    X(6);
                    (3*(7*dth1^2*AJ.l^2*AJ.m*sin(th2) - (3*AJ.tau*cos(2*th1 + 2*th2))/2 - (15*AJ.tau*cos(th2))/2 - (9*AJ.tau*cos(2*th1 + th2))/2 - (3*AJ.mu*AJ.sgn_slip*AJ.tau*sin(2*th1 + 2*th2))/2 - (13*AJ.tau)/2 + 7*dth2^2*AJ.l^2*AJ.m*sin(th2) + 3*dth1^2*AJ.l^2*AJ.m*sin(2*th1 + th2) + 3*dth2^2*AJ.l^2*AJ.m*sin(2*th1 + th2) - (27*AJ.g*AJ.l*AJ.m*cos(th1))/2 + (9*AJ.mu*AJ.sgn_slip*AJ.tau*sin(th2))/2 + 9*dth1^2*AJ.l^2*AJ.m*sin(2*th1) + 3*dth1^2*AJ.l^2*AJ.m*sin(2*th2) + (3*AJ.g*AJ.l*AJ.m*cos(th1 + 2*th2))/2 - (9*AJ.mu*AJ.sgn_slip*AJ.tau*sin(2*th1 + th2))/2 + 9*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip + 14*dth1*dth2*AJ.l^2*AJ.m*sin(th2) + 6*dth1*dth2*AJ.l^2*AJ.m*sin(2*th1 + th2) + 3*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) + 3*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) - 3*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2) - 3*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2) - (27*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1))/2 - 9*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1) + (3*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + 2*th2))/2 + 6*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) - 6*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2)))/(AJ.l^2*AJ.m*(27*cos(2*th1) - 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) + 27*AJ.mu*AJ.sgn_slip*sin(2*th1) - 3*AJ.mu*AJ.sgn_slip*sin(2*th1 + 2*th2) + 41));
                    X(8);
                     -(3*(20*dth1^2*AJ.l^2*AJ.m*sin(th2) - (27*AJ.tau*cos(2*th1))/2 - (3*AJ.tau*cos(2*th1 + 2*th2))/2 - 15*AJ.tau*cos(th2) - 9*AJ.tau*cos(2*th1 + th2) - (3*AJ.mu*AJ.sgn_slip*AJ.tau*sin(2*th1 + 2*th2))/2 - 25*AJ.tau + 7*dth2^2*AJ.l^2*AJ.m*sin(th2) + (AJ.g*AJ.l*AJ.m*cos(th1 + th2))/2 + 6*dth1^2*AJ.l^2*AJ.m*sin(2*th1 + th2) + 3*dth2^2*AJ.l^2*AJ.m*sin(2*th1 + th2) - (27*AJ.g*AJ.l*AJ.m*cos(th1))/2 + 9*dth1^2*AJ.l^2*AJ.m*sin(2*th1) + 6*dth1^2*AJ.l^2*AJ.m*sin(2*th2) + 3*dth2^2*AJ.l^2*AJ.m*sin(2*th2) - (9*AJ.g*AJ.l*AJ.m*cos(th1 - th2))/2 + (3*AJ.g*AJ.l*AJ.m*cos(th1 + 2*th2))/2 - 9*AJ.mu*AJ.sgn_slip*AJ.tau*sin(2*th1 + th2) + dth1^2*AJ.l^2*AJ.m*sin(2*th1 + 2*th2) + dth2^2*AJ.l^2*AJ.m*sin(2*th1 + 2*th2) - (27*AJ.mu*AJ.sgn_slip*AJ.tau*sin(2*th1))/2 + 10*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip + dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip + 14*dth1*dth2*AJ.l^2*AJ.m*sin(th2) + 6*dth1*dth2*AJ.l^2*AJ.m*sin(2*th1 + th2) + 6*dth1*dth2*AJ.l^2*AJ.m*sin(2*th2) + 2*dth1*dth2*AJ.l^2*AJ.m*sin(2*th1 + 2*th2) + 6*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) + 3*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) + (AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + th2))/2 + 2*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip - 6*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2) - 3*dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2) - (27*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1))/2 - 9*dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1) - (9*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 - th2))/2 + (3*AJ.g*AJ.l*AJ.m*AJ.mu*AJ.sgn_slip*sin(th1 + 2*th2))/2 - dth1^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + 2*th2) - dth2^2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + 2*th2) + 6*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(th2) - 6*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + th2) - 2*dth1*dth2*AJ.l^2*AJ.m*AJ.mu*AJ.sgn_slip*cos(2*th1 + 2*th2)))/(AJ.l^2*AJ.m*(27*cos(2*th1) - 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) + 27*AJ.mu*AJ.sgn_slip*sin(2*th1) - 3*AJ.mu*AJ.sgn_slip*sin(2*th1 + 2*th2) + 41))];
            case 'Flight'
                Xdot = [X(2);
                    (72*AJ.tau*sin(th1) - 96*AJ.tau*sin(th1 + th2) - 36*AJ.tau*sin(th1 + th2)*cos(th2) + 108*AJ.tau*cos(th2)*sin(th1) + 81*AJ.tau*sin(th1 + th2)*cos(th1)^2 - 27*AJ.tau*cos(th1 + th2)^2*sin(th1) - 128*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2) - 128*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2) + 27*AJ.tau*cos(th1 + th2)*sin(th1 + th2)*cos(th1) - 384*dth1^2*AJ.l^2*AJ.m*cos(th1) - 81*AJ.tau*cos(th1 + th2)*cos(th1)*sin(th1) + 48*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^3 + 48*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^3 + 324*dth1^2*AJ.l^2*AJ.m*cos(th1)^3 + 192*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th2) - 256*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2) + 48*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2 + 48*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2 - 144*dth1^2*AJ.l^2*AJ.m*sin(th1)*sin(th2) - 144*dth2^2*AJ.l^2*AJ.m*sin(th1)*sin(th2) + 108*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)^2 + 144*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1) + 72*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)^2 + 108*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)^2 + 72*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)^2 + 216*dth1^2*AJ.l^2*AJ.m*cos(th1)*cos(th2)^2 + 96*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^3 + 324*dth1^2*AJ.l^2*AJ.m*cos(th1)*sin(th1)^2 - 216*dth1^2*AJ.l^2*AJ.m*cos(th2)*sin(th1)*sin(th2) - 324*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)^2*cos(th2) - 108*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*cos(th2) - 108*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*cos(th2) - 162*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th1)^2 - 54*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*cos(th2) - 54*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*cos(th2) + 96*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2 - 288*dth1*dth2*AJ.l^2*AJ.m*sin(th1)*sin(th2) - 162*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)^2*sin(th2) + 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1)*sin(th2) + 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1)*sin(th2) + 216*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)^2 + 144*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)^2 + 144*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1) + 108*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*sin(th1) + 108*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*sin(th1) + 72*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)*sin(th2) + 72*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)*sin(th2) - 216*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*cos(th2) - 108*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*cos(th2) + 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1)*sin(th2) - 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th2) - 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th2)*sin(th1) - 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th2) - 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th2)*sin(th1) + 162*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th1)*sin(th2) - 162*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*cos(th2)*sin(th1) + 216*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*sin(th1) + 144*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)*sin(th2) - 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th2) - 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th2)*sin(th1))/(AJ.l*AJ.m*(96*cos(th1 + th2)^2 + 96*sin(th1 + th2)^2 + 216*cos(th1)^2 + 144*cos(th2)^2 + 216*sin(th1)^2 - 81*cos(th1 + th2)^2*sin(th1)^2 - 81*sin(th1 + th2)^2*cos(th1)^2 - 216*cos(th1 + th2)*cos(th1)*cos(th2) - 216*sin(th1 + th2)*cos(th2)*sin(th1) + 162*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th1) - 256));
                    X(4);
                    (96*AJ.tau*cos(th1 + th2) - 72*AJ.tau*cos(th1) + 36*AJ.tau*cos(th1 + th2)*cos(th2) - 108*AJ.tau*cos(th1)*cos(th2) + 256*AJ.g*AJ.l*AJ.m - 81*AJ.tau*cos(th1 + th2)*sin(th1)^2 + 27*AJ.tau*sin(th1 + th2)^2*cos(th1) - 128*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2) - 128*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2) - 27*AJ.tau*cos(th1 + th2)*sin(th1 + th2)*sin(th1) - 384*dth1^2*AJ.l^2*AJ.m*sin(th1) + 81*AJ.tau*sin(th1 + th2)*cos(th1)*sin(th1) + 48*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^3 + 48*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^3 + 324*dth1^2*AJ.l^2*AJ.m*sin(th1)^3 - 96*AJ.g*AJ.l*AJ.m*cos(th1 + th2)^2 - 96*AJ.g*AJ.l*AJ.m*sin(th1 + th2)^2 - 216*AJ.g*AJ.l*AJ.m*cos(th1)^2 - 144*AJ.g*AJ.l*AJ.m*cos(th2)^2 - 216*AJ.g*AJ.l*AJ.m*sin(th1)^2 + 144*dth1^2*AJ.l^2*AJ.m*cos(th1)*sin(th2) + 144*dth2^2*AJ.l^2*AJ.m*cos(th1)*sin(th2) - 256*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2) + 48*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2) + 48*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2) + 72*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)^2 + 72*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)^2 + 108*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)^2 + 144*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th1) + 108*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)^2 + 324*dth1^2*AJ.l^2*AJ.m*cos(th1)^2*sin(th1) + 216*dth1^2*AJ.l^2*AJ.m*cos(th2)^2*sin(th1) + 96*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^3 - 192*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th2) + 81*AJ.g*AJ.l*AJ.m*cos(th1 + th2)^2*sin(th1)^2 + 81*AJ.g*AJ.l*AJ.m*sin(th1 + th2)^2*cos(th1)^2 + 216*AJ.g*AJ.l*AJ.m*sin(th1 + th2)*cos(th2)*sin(th1) + 288*dth1*dth2*AJ.l^2*AJ.m*cos(th1)*sin(th2) - 162*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)^2*cos(th2) - 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th2)*sin(th1) - 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th2)*sin(th1) + 96*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2) + 162*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1)^2*sin(th2) - 324*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)*sin(th1)^2 - 54*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th2) - 108*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th2)*sin(th1) - 54*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th2) - 108*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th2)*sin(th1) + 144*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)^2 + 216*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)^2 + 144*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1) + 108*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th1) + 108*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th1) - 72*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th2) - 72*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th2) + 216*dth1^2*AJ.l^2*AJ.m*cos(th1)*cos(th2)*sin(th2) + 216*AJ.g*AJ.l*AJ.m*cos(th1 + th2)*cos(th1)*cos(th2) - 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th2)*sin(th1) - 108*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th2) - 216*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th2)*sin(th1) - 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*cos(th2) - 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*cos(th2) + 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)*sin(th2) + 54*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)*sin(th2) - 162*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*cos(th2)*sin(th1) - 162*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*sin(th1)*sin(th2) - 162*AJ.g*AJ.l*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th1) + 216*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th1) - 144*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th2) - 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*cos(th2) + 108*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)*sin(th2))/(AJ.l*AJ.m*(96*cos(th1 + th2)^2 + 96*sin(th1 + th2)^2 + 216*cos(th1)^2 + 144*cos(th2)^2 + 216*sin(th1)^2 - 81*cos(th1 + th2)^2*sin(th1)^2 - 81*sin(th1 + th2)^2*cos(th1)^2 - 216*cos(th1 + th2)*cos(th1)*cos(th2) - 216*sin(th1 + th2)*cos(th2)*sin(th1) + 162*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th1) - 256));
                    X(6);
                    (3*(16*AJ.tau + 24*AJ.tau*cos(th2) - 6*AJ.tau*cos(th1 + th2)^2 - 6*AJ.tau*sin(th1 + th2)^2 - 18*AJ.tau*cos(th1 + th2)*cos(th1) - 18*AJ.tau*sin(th1 + th2)*sin(th1) - 32*dth1^2*AJ.l^2*AJ.m*sin(th2) - 32*dth2^2*AJ.l^2*AJ.m*sin(th2) - 48*dth1^2*AJ.l^2*AJ.m*cos(th2)*sin(th2) - 64*dth1*dth2*AJ.l^2*AJ.m*sin(th2) + 12*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 9*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 12*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 9*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 12*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) + 12*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) - 24*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 24*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) - 24*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 24*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) + 27*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)^2 + 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1) + 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1) + 27*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*sin(th1) - 27*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th1) + 24*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 18*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 24*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) + 36*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th2) - 36*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th1) + 36*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*cos(th2) + 36*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)*sin(th2) - 48*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 48*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) - 27*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)^2 - 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) - 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) - 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) + 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1)))/(AJ.l^2*AJ.m*(96*cos(th1 + th2)^2 + 96*sin(th1 + th2)^2 + 216*cos(th1)^2 + 144*cos(th2)^2 + 216*sin(th1)^2 - 81*cos(th1 + th2)^2*sin(th1)^2 - 81*sin(th1 + th2)^2*cos(th1)^2 - 216*cos(th1 + th2)*cos(th1)*cos(th2) - 216*sin(th1 + th2)*cos(th2)*sin(th1) + 162*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th1) - 256));
                    X(8);
                    -(3*(80*AJ.tau - 54*AJ.tau*cos(th1)^2 - 54*AJ.tau*sin(th1)^2 + 48*AJ.tau*cos(th2) - 6*AJ.tau*cos(th1 + th2)^2 - 6*AJ.tau*sin(th1 + th2)^2 - 36*AJ.tau*cos(th1 + th2)*cos(th1) - 36*AJ.tau*sin(th1 + th2)*sin(th1) - 160*dth1^2*AJ.l^2*AJ.m*sin(th2) - 32*dth2^2*AJ.l^2*AJ.m*sin(th2) - 96*dth1^2*AJ.l^2*AJ.m*cos(th2)*sin(th2) - 48*dth2^2*AJ.l^2*AJ.m*cos(th2)*sin(th2) - 64*dth1*dth2*AJ.l^2*AJ.m*sin(th2) + 81*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1)^3 - 81*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)^3 + 12*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 9*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 12*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 9*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 12*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) + 12*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) + 108*dth1^2*AJ.l^2*AJ.m*cos(th1)^2*sin(th2) + 108*dth1^2*AJ.l^2*AJ.m*sin(th1)^2*sin(th2) - 120*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 120*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) - 24*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 24*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) + 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)^2 + 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1) + 27*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)^2 + 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1) - 96*dth1*dth2*AJ.l^2*AJ.m*cos(th2)*sin(th2) + 81*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)^2*sin(th1) + 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*sin(th1) + 27*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*sin(th1) - 81*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*sin(th1)^2 - 54*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th1) - 27*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th1) + 24*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th2) + 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^3*sin(th1) - 18*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^3*cos(th1) + 24*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*sin(th2) + 72*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th2) - 72*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th1) + 72*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*cos(th2) + 36*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th2) - 36*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th1) + 36*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*cos(th2) + 72*dth1^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)*sin(th2) + 36*dth2^2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)*sin(th2) - 48*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1) + 48*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1) - 54*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)^2 - 9*dth1^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) - 27*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)^2 - 9*dth2^2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) - 54*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)^2 - 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1 + th2)*cos(th1) + 54*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*sin(th1)^2 + 18*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)^2*sin(th1) + 54*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)^2*cos(th1)*sin(th1) - 54*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)*sin(th1) + 72*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*sin(th2) - 72*dth1*dth2*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th2)*sin(th1) + 72*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th1)*cos(th2) + 72*dth1*dth2*AJ.l^2*AJ.m*sin(th1 + th2)*sin(th1)*sin(th2)))/(96*AJ.l^2*AJ.m*cos(th1 + th2)^2 - 256*AJ.l^2*AJ.m + 96*AJ.l^2*AJ.m*sin(th1 + th2)^2 + 216*AJ.l^2*AJ.m*cos(th1)^2 + 144*AJ.l^2*AJ.m*cos(th2)^2 + 216*AJ.l^2*AJ.m*sin(th1)^2 - 81*AJ.l^2*AJ.m*cos(th1 + th2)^2*sin(th1)^2 - 81*AJ.l^2*AJ.m*sin(th1 + th2)^2*cos(th1)^2 - 216*AJ.l^2*AJ.m*sin(th1 + th2)*cos(th2)*sin(th1) - 216*AJ.l^2*AJ.m*cos(th1 + th2)*cos(th1)*cos(th2) + 162*AJ.l^2*AJ.m*cos(th1 + th2)*sin(th1 + th2)*cos(th1)*sin(th1))];
        end
        end
        
        function [value, isterminal, direction] = Events(AJ, X)
            value = ones(AJ.nEvents,1);
            isterminal = ones(AJ.nEvents,1);
            direction = [-1, -1, -1, -1, -1, -1, -1, 0, 0, -1].';
            switch AJ.Phase
                case 'Stick'
                    % Event 1 - Stick -> Slip
                    [Ft, Fn] = GetReactionForces(AJ, X);
                    value(1) =  abs(AJ.mu*Fn) - abs(Ft);
                    % Event 3 - Liftoff!
                    value(3) = Fn;
                    % Event 5 - Q collision
                    Q = GetPos(AJ, X, 'Q');
                    value(5) = Q(2);
                    % Event 6 - R collision
                    R = GetPos(AJ, X, 'R');
                    value(6) = R(2);
                    if ~AJ.landed % Check the following only before landing
                        % Event 7 - Too much rotation
                        Q = GetPos(AJ, X, 'Q');
                        P = GetPos(AJ, X, 'P');
                        R = GetPos(AJ, X, 'R');
                        RmP = R - P;
                        value(7) = pi/3 - abs(atan(RmP(1)/RmP(2)));
                        % Event 8 - Q isn't below R
                        value(8) = R(2) - Q(2);
                        % Event 9 - Q isn't above P
                        value(9) = Q(2) - P(2);
                    end
                    % Event 10 - abs(th2)<180
                    value(10) = pi - abs(X(7));
                case 'Slip'
                    % Event 2 - Slip -> Stick
                    value(2) = X(2)/AJ.sgn_slip;
                    % Event 3 - Liftoff!
                    [~, Fn] = GetReactionForces(AJ, X);
                    value(3) = Fn;
                    % Event 5 - Q collision
                    Q = GetPos(AJ, X, 'Q');
                    value(5) = Q(2);
                    % Event 6 - R collision
                    R = GetPos(AJ, X, 'R');
                    value(6) = R(2);
                    if ~AJ.landed % Check the following only before landing
                        % Event 7 - Too much rotation
                        Q = GetPos(AJ, X, 'Q');
                        P = GetPos(AJ, X, 'P');
                        R = GetPos(AJ, X, 'R');
                        RmP = R - P;
                        value(7) = pi/3 - abs(atan(RmP(1)/RmP(2)));
                        % Event 8 - Q isn't below R
                        value(8) = R(2) - Q(2);
                        % Event 9 - Q isn't above P
                        value(9) = Q(2) - P(2);
                    end
                    % Event 10 - abs(th2)<180
                    value(10) = pi - abs(X(7));
                case 'Flight'
                    % Event 4 - P collision
                    value(4) = X(3);
                    % Event 5 - Q collision
                    Q = GetPos(AJ, X, 'Q');
                    value(5) = Q(2);
                    % Event 6 - R collision
                    R = GetPos(AJ, X, 'R');
                    value(6) = R(2);
                    if ~AJ.landed % Check the following only before landing
                        % Event 7 - Too much rotation
                        P = GetPos(AJ, X, 'P');
                        RmP = R - P;
                        value(7) = pi/3 - abs(atan(RmP(1)/RmP(2)));
                        % Event 8 - Q isn't below R
                        value(8) = R(2) - Q(2);
                        % Event 9 - Q isn't above P
                        value(9) = Q(2) - P(2);
                    end
                    % Event 10 - abs(th2)<180 
                    value(10) = pi - abs(X(7)); 
            end 
        end
        
        function [Xf] = impact_law(AJ, Xi)
        x = Xi(1); dx = Xi(2); y = Xi(3); dy = Xi(4); th1 = Xi(5); dth1 = Xi(6); %#ok
        th2 = Xi(7); dth2 = Xi(8); %#ok
        
        M = [                                  2*AJ.m,                                      0, -AJ.l*AJ.m*(sin(th1 + th2) + 3*sin(th1)),          -AJ.l*AJ.m*sin(th1 + th2);
            0,                                 2*AJ.m,  AJ.l*AJ.m*(cos(th1 + th2) + 3*cos(th1)),           AJ.l*AJ.m*cos(th1 + th2);
            -AJ.l*AJ.m*(sin(th1 + th2) + 3*sin(th1)), AJ.l*AJ.m*(cos(th1 + th2) + 3*cos(th1)),       (4*AJ.l^2*AJ.m*(3*cos(th2) + 5))/3, (2*AJ.l^2*AJ.m*(3*cos(th2) + 2))/3;
            -AJ.l*AJ.m*sin(th1 + th2),                AJ.l*AJ.m*cos(th1 + th2),       (2*AJ.l^2*AJ.m*(3*cos(th2) + 2))/3,                  (4*AJ.l^2*AJ.m)/3];
        
        W = [1 0 0 0; 0 1 0 0 ];
        A = W*inv(M)*W.';
        
%         A = [ (4*(27*cos(2*th1) + 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) - 41))/(AJ.m*(18*cos(2*th2) - 82)),                          (6*(9*sin(2*th1) - sin(2*th1 + 2*th2)))/(AJ.m*(9*cos(2*th2) - 41));
%             (6*(9*sin(2*th1) - sin(2*th1 + 2*th2)))/(AJ.m*(9*cos(2*th2) - 41)), -(4*(27*cos(2*th1) - 9*cos(2*th2) - 3*cos(2*th1 + 2*th2) + 41))/(AJ.m*(18*cos(2*th2) - 82))];
        
    
        Lambda1 = [0 -Xi(4)/A(2,2)].'; Lambda2 = -inv(A)*([Xi(2) Xi(4)].');
        if AJ.mu*Lambda2(2) >= abs(Lambda2(1))
            Kappa = 1;
        else
            Kappa = AJ.mu*Lambda1(2)/(abs(Lambda2(1)) - AJ.mu*(Lambda2(2) - Lambda1(2)));
        end
        Lambda = Lambda1 + Kappa*(Lambda2 - Lambda1);
        Xf = Xi;
        sol = inv(M)*W.'*Lambda;
        Xf(2) = Xi(2) + sol(1);
        Xf(4) = Xi(4) + sol(2);
        Xf(6) = Xi(6) + sol(3);
        Xf(8) = Xi(8) + sol(4);
        end
       
        function [Xf] = HandleEvent(AJ, iEvent, Xi)
            switch iEvent(end)
                case 1 % Stick -> Slip
                    [Ft, ~] = GetReactionForces(AJ, Xi);
                    AJ.sgn_slip = -sign(Ft);
                    Xf = Xi;
                    AJ.Phase = 'Slip';
                case 2 % Slip -> Stick
                    Xf = Xi;
                    AJ.Phase = 'Stick';
                case 3 % Liftoff
                    Xf = Xi;
                    if ~AJ.jumped 
                        AJ.LiftOff = Xi(1); % Save First Liftoff Point
                    else
                        AJ.jumpedAgain = 1;
                    end
                    AJ.jumped = 1;
                    AJ.Phase = 'Flight';
                case 4 % P collision
                    Xf = impact_law(AJ, Xi);
                    if abs(Xf(2)) > 1e-14
                        AJ.Phase = 'Slip';
                        AJ.sgn_slip = sign(Xf(2));
                        [~, Fn] = AJ.GetReactionForces(Xf);
                        if Fn < 0
                            AJ.Phase = 'Flight';
                            AJ.jumpedAgain = 1;
                        end
                    else
                        AJ.Phase = 'Stick';
                        [Ft, Fn] = AJ.GetReactionForces(Xf); 
                        if Fn < 0
                            AJ.Phase = 'Flight';
                            AJ.jumpedAgain = 1;
                        else
                            if abs(Ft) >= AJ.mu*abs(Fn)
                                AJ.sgn_slip = -sign(Ft);
                                AJ.Phase = 'Slip';
                            end
                        end
                    end
                    AJ.landed = 1;
                case 5 % Q collision
                    LandPos =  AJ.GetPos(Xi, 'Q');
                    AJ.LandingQR = LandPos(1);
                    Xf = Xi;
                case 6 % R collision
                    LandPos =  AJ.GetPos(Xi, 'R');
                    AJ.LandingQR = LandPos(1);
                    Xf = Xi;
                case 7 % Too much rotation
                    Xf = Xi;
                case 8 % Q isn't below R
                    Xf = Xi;
                case 9 % Q isn't above P
                    Xf = Xi;
                case 10 % abs(th2)<180
                    Xf = Xi;
            end
        end     
    end
end