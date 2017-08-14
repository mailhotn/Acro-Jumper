nParams = 4;
LB = [pi/18    ,-pi+0.001,0  ,-1];
UB = [17/18*pi ,pi-0.001 ,0.1,1];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GASim,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['Workspaces/GAsol_LO_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');