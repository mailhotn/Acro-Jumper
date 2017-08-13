nParams = 13;
LB = [pi/18 ,-pi,0,0,0,-10*ones(1,8)];
UB = [17/18*pi ,pi, 0.3, 0.4, 0.6, 10*ones(1,8)];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv});
GAsol = ga(@GASim,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['GAsol' num2str(c(3)) num2str(c(4)) num2str(c(5))],'GAsol');