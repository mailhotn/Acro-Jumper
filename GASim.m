function fitness = GASim(Params)
% GASim simulates the AcroJumper and returns the fitness of the controller

liftoff = load('Workspaces/GAsol_fit-4.1354_d14_h11_m41.mat');
liftoff = liftoff.GAsol;
AJ = AcroJumper;

Control = ControllerOrd2Seg([0.054472931646088, Params(1:2)],-10,Params(3),Params(4),Params(5));
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 liftoff(1) 0 liftoff(2) 0].';
Sim.GetInitPhase;

R = Sim.Mod.GetPos(Sim.IC,'R'); % check if Initial conditions make sense
if R(2) < 0
    fitness = 100;
    return
end

opt = odeset('reltol', 1e-12, 'abstol', 1e-12, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
if Ie(end) >= 5
    EndCond = 1;
end
while ~EndCond
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    if Ie(end) >= 5
        Sim.Mod.HandleEvent(Ie(end),X(end,:));
        EndCond = 1;
    end
end
fitness = GetFit(AJ, Control, X, Time, Te, Ie);

end

