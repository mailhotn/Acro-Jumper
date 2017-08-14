nParams = 5;
LB = [0.054472931646088,0.054472931646088,-10*ones(1,3)];
UB = [1,1,10*ones(1,3)];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GASim,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');