AJ = AcroJumper;
AJF = AcroJumper;
Ft = []; Fn = []; COM = [];
Control = ControllerOrd2Seg(Params(3:5),Params(6),Params(7),Params(8),Params(9));
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 Params(1) 0 Params(2) 0].'; 
Sim.GetInitPhase;
opt = odeset('reltol', 1e-9, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
for ii = 1:length(X(:,1))
    AJF = Sim.Mod.copy;
    AJF.tau = Sim.Con.calc_tau(Time(ii));
    [Ft(ii) Fn(ii)] = AJF.GetReactionForces(X(ii,:)); %#ok
    COM(ii,:) = AJF.GetPos(X(ii,:),'CM'); %#ok
end
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
if ~Sim.Mod.landed
    Ie(Ie == 4) = [];
end
if Ie(end) >= 5
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    for ii = 1:length(tX(:,1))
        AJF = Sim.Mod.copy;
        AJF.tau = Sim.Con.calc_tau(tTime(ii));
        [tFt(ii) tFn(ii)] = AJF.GetReactionForces(tX(ii,:)); %#ok
        tCOM(ii,:) = AJF.GetPos(tX(ii,:),'CM'); %#ok
    end
    COM = [COM; tCOM]; %#ok
    tCOM = [];
    Ft = [Ft, tFt]; Fn = [Fn, tFn]; %#ok
    tFt = []; tFn = []; tX = []; tTime = [];
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    if ~Sim.Mod.landed
        Ie(Ie == 4) = [];
    end
    if Ie(end) >= 5
        EndCond = 1;
    end
end
%% Animate simulation
figure(2)
path = animatedline('color', 'b', 'linewidth', 2);
for ii = 1:length(Time)-1
    Render(AJ,Time(ii),X(ii,:),1);
    dt = Time(ii+1) - Time(ii);
    addpoints(path, COM(ii,1), COM(ii,2));
    drawnow;
%     pause(dt);
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

