lb = [pi/18 -pi 0 0 0 -10 -10 -10 -10]; ub = [17/18*pi pi 0.1 0.1 0.1 10 10 10 10];
x0 = [pi/3 5/6*pi 0.05 0.07 0.1 0 0 0 0];

opt = optimoptions('patternsearch','PlotFcn',@psplotbestf);

sol = patternsearch(@GASim,x0,[],[],[],[],lb,ub,opt);