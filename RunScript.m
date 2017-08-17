AJ = AcroJumper;
AJF = AcroJumper;
Ft = []; Fn = []; COM = [];
Control = ControllerOrd2Seg(Params(3:5),Params(6),Params(7),Params(8),Params(9));
Sim = Simulation(AJ, Control);

Sim.IC = [0 0 0 0 Params(1) 0 Params(2) 0].'; 
Sim.GetInitPhase;
opt = odeset('reltol', 1e-9, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
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
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xe = [Xe; tXe]; %#ok
    for ii = 1:length(tX(:,1))
        AJF = Sim.Mod.copy;
        AJF.tau = Sim.Con.calc_tau(tTime(ii));
        [tFt(ii) tFn(ii)] = AJF.GetReactionForces(tX(ii,:)); %#ok
        tCOM(ii,:) = AJF.GetPos(tX(ii,:),'CM'); %#ok
    end
    COM = [COM; tCOM]; %#ok
    tCOM = [];
    Ft = [Ft, tFt]; Fn = [Fn, tFn]; %#ok
    tFt = []; tFn = []; tX = []; tTime = []; tXe = [];
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
    if ~Sim.Mod.landed
        Ie(Ie == 4) = [];
    end
    if Ie(end) >= 5
        EndCond = 1;
    end
end
%% Get Distance
LandEventInd = find(Ie == 4);                        % the index of the landing event
if ~isempty(LandEventInd)
    LandTime = Te(LandEventInd(1));                  % get the time of landing
    LandTimeInd = find(abs(Time-LandTime) < 1e-6);   % find the timestep index of the land time
    Pmin = min(X(LandTimeInd:end,1));                % find the minimal coordinate of P after landing time
    d = min([Pmin, AJ.LandingQR]) - AJ.LiftOff;
    disp(['Jump Length: ' num2str(d*100) ' centimeters'])
end
%% Animate simulation
load handel
sound(y,Fs)
figure(2)
% path = animatedline('color', 'b', 'linewidth', 2);
for ii = 1:length(Time)-1
    Render(AJ,Time(ii),X(ii,:),1);
    dt = Time(ii+1) - Time(ii);
%     addpoints(path, COM(ii,1), COM(ii,2));
%     drawnow;
    pause(dt*11);
end
%% Draw Graphs For Report
figure(3)
tau = Time;
for ii = 1:length(Time)
    tau(ii) = Sim.Con.calc_tau(Time(ii));
    AJ.tau = tau(ii);
end
plot(Time,tau,'Linewidth',1.3)
h = vline(Te);
set(h,'Color',[0.3,0.3,0.3])
% title('\tau Profile')
xlabel('Time [sec]')
ylabel('\tau [Nm]')

a = figure(4);
plot([0,Time(end)],[-AJ.mu -AJ.mu; AJ.mu AJ.mu].',':r',Time, Ft./Fn,'Linewidth',1.3)
ylim([-0.4,0.4])
% title('Reaction Force Ratio')
xlabel('Time [sec]')
ylabel('F_t/F_n')
h = vline(Te);
set(h,'Color',[0.3,0.3,0.3])

b = figure(5);
plot(Time, Fn,[0,Time(end)],[0 0],':r','Linewidth',1.3)
% title('Normal Reaction Force')
xlabel('Time [sec]')
ylabel('F_n [N]')
h = vline(Te);
set(h,'Color',[0.3,0.3,0.3])
b.Position = a.Position;

figure(6)
plot(Time, [X(:,1),X(:,3)],'Linewidth',1.3)
xlabel('Time [sec]')
ylabel('Position [m]')
legend('x','y','Location','best')
h = vline(Te);
set(h,'Color',[0.3,0.3,0.3])

figure(7)
plot(Time,[X(:,5)*180/pi,X(:,7)*180/pi],'Linewidth',1.3)
xlabel('Time [sec]')
ylabel('Angle [deg]')
legend('\theta_1','\theta_2','Location','best')
h = vline(Te);
set(h,'Color',[0.3,0.3,0.3])

drawX = [Sim.IC.';Xe(1,:);X(781,:);X(896,:);X(961,:);Xe(2,:);X(1097,:);X(1117,:);Xe(3,:)];
figure(8)
for ii =1:9
    AJs{ii} = AcroJumper;
    AJs{ii}.LineWidth = 0.005;
    AJs{ii}.link_width = 0.01;
    Render(AJs{ii},[],drawX(ii,:),1);
    hold on
end
xlabel('x [m]')
ylabel('y [m]')
