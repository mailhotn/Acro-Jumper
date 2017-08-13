function state = GARender(options, state, flag)
AJ = AcroJumper;
AJ.Phase = 'Stick';
% Control = Controller(Params(3), Params(4), Params(5:8),Params(9:10),Params(11:12)...
%         ,Params(13), Params(14:17),Params(18:19),Params(20:21));
    
%     Control = ControllerF(Params(3:21));
Params = state.Best
Control = ControllerOrd2Seg(Params(3:5),Params(6:8),Params(9:11),Params(12:14),Params(15:17));
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 Params(1) 0 Params(2) 0].';
   
opt = odeset('reltol', 1e-8, 'abstol', 1e-8, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
Sim.Mod.Phase
Time(end)
if Ie(end) >= 5
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; %#ok
    Te = [Te; tTe]; %#ok
    X = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    Sim.Mod.Phase
    Time(end)
    if Ie(end) >= 5
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

end