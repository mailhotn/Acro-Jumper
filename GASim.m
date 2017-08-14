function fitness = GASim(Params)
% GASim simulates the AcroJumper and returns the fitness of the controller

% liftoff = load('Workspaces/GAsol_LO_fit-1.8175_d14_h13_m46.mat');
% liftoff = liftoff.GAsol;
AJ = AcroJumper;
Control = ControllerOrd2Seg(Params(3:5),Params(6),Params(7),Params(8),Params(9));
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
if Ie(end) >= 5 || AJ.Painleve
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    if Ie(end) >= 5 || AJ.Painleve
        EndCond = 1;
    end
end
fitness = GetFit(AJ, Control, X, Time, Te, Ie);
end

