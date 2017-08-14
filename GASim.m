function fitness = GASim(Params)
% GASim simulates the AcroJumper and returns the fitness of the controller

AJ = AcroJumper;
% Control = Controller(Params(3), Params(4), Params(5:8),Params(9:10),Params(11:12)...
%         ,Params(13), Params(14:17),Params(18:19),Params(20:21));
        
% Control = ControllerF(Params(3:21));

Control = ControllerOrd2Seg([Params(3),Inf,Inf],10*sign(Params(4)),[],[],[]);
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 Params(1) 0 Params(2) 0].';
Sim.GetInitPhase;

R = Sim.Mod.GetPos(Sim.IC,'R'); % check if Initial conditions make sense
if R(2) < 0
    fitness = 100;
    return
end

opt = odeset('reltol', 1e-12, 'abstol', 1e-12, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
if Ie(end) >= 3
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Sim.Mod.HandleEvent(Ie(end),X(end,:));
    if Ie(end) >= 3
        EndCond = 1;
    end
end
fitness = FitLiftOff(AJ,X(end,:));
end

