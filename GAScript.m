nParams = 9;
LB = [pi/6    ,-pi+0.001,0,0,0,-10*ones(1,4)];
UB = [5/6*pi ,pi-0.001 , 0.2, 0.4, 1, 10*ones(1,4)];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GASim,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');