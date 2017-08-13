syms m l g tau sigma mu real
syms  x y th1 th2 real
syms  dx dy dth1 dth2 real
syms  ddx ddy ddth1 ddth2 real

q = [x y th1 th2].';
dq = [dx dy dth1 dth2].';
ddq = [ddx ddy ddth1 ddth2].';
I = 1/3*m*l^2;
grav = [0 -g].';

%% Find Energy
P = [x,y].';
Q = P + [2*l*cos(th1),2*l*sin(th1)].';

x1 = P + [l*cos(th1),l*sin(th1)].';
x2 = Q + [l*cos(th1 + th2),l*sin(th1 + th2)].';

v1 = jacobian(x1,q)*dq;
v2 = jacobian(x2,q)*dq;
w1 = dth1;
w2 = dth1 + dth2;

T = 0.5*m*(v1.'*v1 + v2.'*v2) + 0.5*I*(w1^2 + w2^2);
V = -m*dot(x1,grav)-m*dot(x2,grav);
%% Get Matrices
M = simplify(hessian(T,dq));
G = simplify(jacobian(V,q)).';
for ii = 1:4
    Mdot(:,ii) = jacobian(M(:,ii),q)*dq;
end
for ii = 1:4
    Mjac = diff(M,q(ii));
    M2(:,ii) = Mjac*dq;
end
C = simplify(Mdot-0.5*M2.');
Fq = [0 0 0 tau].';

%% Free Flight
W = 0;
eqs_flight = M*ddq +G +C*dq == Fq;
sol_flight = solve(eqs_flight,ddq);
%% Constraints
Wt = [1 0 0 0];
Wn = [0 1 0 0];
%% Stick
W = [Wn; Wt];
lambda = (W*M^-1*W.')^-1*(W*M^-1*(C*dq+G-Fq));

eqs_stick = M*ddq +G +C*dq ==  W.'*lambda + Fq;
sol_stick = solve(eqs_stick,ddq);
%% Slip
A = simplify((Wn*M^-1)*(C*dq+G-Fq));
B = simplify((Wn*M^-1)*(Wn-sigma*mu*Wt).');
lambdan = A/B;
eqs_slip = M*ddq + C*dq + G == (Wn - mu*sigma*Wt).'*lambdan + Fq;
sol_slip = solve(eqs_slip,ddq);
%% Impact Law
Aim = simplify(W*M^-1*W.');
