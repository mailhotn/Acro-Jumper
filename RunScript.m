AJ = AcroJumper;
Ft = []; Fn = [];
liftoff = load('Workspaces/GAsol_fit-4.1354_d14_h11_m41.mat');
liftoff = liftoff.GAsol;

Control = ControllerOrd2Seg([0.0558, Params(1:2)],-10,Params(3),Params(4),Params(5));
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 liftoff(1) 0 liftoff(2) 0].'; 
Sim.GetInitPhase;
opt = odeset('reltol', 1e-12, 'abstol', 1e-12, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
for ii = 1:length(X)
    AJ.tau = Sim.Con.calc_tau(Time(ii));
    [Ft(ii) Fn(ii)] = AJ.GetReactionForces(X(ii,:)); %#ok
end
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
if Ie(end) >= 3
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    for ii = 1:length(tX(:,1))
        AJ.tau = Sim.Con.calc_tau(tTime(ii));
        [tFt(ii) tFn(ii)] = AJ.GetReactionForces(tX(ii,:)); %#ok
    end
    Ft = [Ft, tFt]; Fn = [Fn, tFn]; %#ok
    tFt = []; tFn = []; tX = []; tTime = [];
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    if Ie(end) >= 3
        EndCond = 1;
    end
end
%% Animate simulation
figure(2)
for i = 1:length(Time)-1
    Render(AJ,Time(i),X(i,:),1);
    dt = Time(i+1) - Time(i);
    pause(dt*10);
end
figure(3)
tau = Time;
for ii = 1:length(Time)
    tau(ii) = Sim.Con.calc_tau(Time(ii));
    AJ.tau = tau(ii);
end
plot(Time,tau)
title('\tau Profile')
xlabel('Time [sec]')
ylabel('\tau [Nm]')
figure(4)
plot(Time, Ft./Fn)
title('Reaction Force Ratio')
xlabel('Time [sec]')
ylabel('F_t/F_n')
figure(5)
plot(Time, Fn)
title('Normal Reaction Force')
xlabel('Time [sec]')
ylabel('F_n [N]')
figure(6)
plot(Time, Ft)
title('Tangent Reaction Force')
xlabel('Time [sec]')
ylabel('F_t [N]')

