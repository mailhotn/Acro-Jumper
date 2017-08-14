nParams = 9;
LB = [pi/18 ,5/9*pi,0,0,0,-10*ones(1,4)];
UB = [pi/3  ,17/18*pi,0.1, 0.2, 0.3 ,0,10*ones(1,3)];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GASim,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');