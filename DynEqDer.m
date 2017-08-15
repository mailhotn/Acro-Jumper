syms mass len mju grav Ic sigma taut real                           % problem's parameters
syms th1 th2 dth1 dth2 ddth1 ddth2 x dx ddx y dy ddy real           % problem's generalized coordinates

Ic = 1/12*mass*(2*len)^2;
q = [x y th1 th2].'; dq = [dx dy dth1 dth2].'; ddq = [ddx ddy ddth1 ddth2].';

%% Unconstraint Dynamics %% 

% Positions and velocities
r1 = [x + len*cos(th1), y + len*sin(th1)];
Q  = [x + 2*len*cos(th1), y + 2*len*sin(th1)];
r2 = Q + [len*cos(th1 + th2), len*sin(th1 + th2)];

v1 = (jacobian(r1, q)*dq).';
v2 = (jacobian(r2, q)*dq).';

vc = (v1 + v2)/2;

aCOM = (jacobian(vc,q)*dq).' + (jacobian(vc,dq)*ddq).';


% Kinetic energy
KE = 1/2*mass*(v1*v1.') + 1/2*mass*(v2*v2.') + 1/2*Ic*dth1^2 + 1/2*Ic*(dth2+dth1)^2;

% Potential energy
PE = -[0, -mass*grav]*r1.' - [0, -mass*grav]*r2.';

% External generalized forces power
Power = taut*dth2;

% Equation's matrices

Fq = jacobian(Power,dq).';         % Generalized forces
G  = simplify(jacobian(PE, q).');  % Potential energy generalized forces vector
M  = simplify(hessian(KE,dq));     % Inertia matrix
C  = sym(zeros(4,1));

for ii = 1:4
    sub(ii) = diff(KE,dq(ii)); %#ok
    for jj = 1:4
        subb = diff(sub(ii),q(jj));
        C(ii) = C(ii) + subb*dq(jj); 
    end
    C(ii) = C(ii) - diff(KE,q(ii));
end
C = simplify(C);                  % C vector

%% Stick Constraint %% 

W = jacobian([x y], q);
Wt = W(1,:);
Wn = W(2,:);
n = size(W);
Wdot = sym(zeros(n));
for i = 1:n(1)
    Wdot(i,:) = (jacobian(W(i,:),q)*dq).';
end
Wdot = simplify(Wdot); 
Wtdot = Wdot(1,:);
Wndot = Wdot(2,:);

a = [M -Wt.' -Wn.';
    Wt zeros(1,1) zeros(1,1);
    Wn zeros(1,1) zeros(1,1)];
b = [Fq - C - G;
     -Wtdot*dq;
     -Wndot*dq];
  
 sol_stick = simplify(a\b);
 
%% Slip Constraint %%

a = [M -Wt.' -Wn.';
    zeros(1,4) 1 sigma*mju;
    Wn zeros(1,1) zeros(1,1)];
b = [Fq - C - G;
    0;
    -Wndot*dq];

sol_slip = simplify(a\b);

%% Free flight

sol_FF = M\(Fq - C - G);

%% Painleve paradox

A = simplify(Wn*M^-1*(C + G - Fq) - Wndot*dq);
B = simplify(Wn*M^-1*((Wn - sigma*mju*Wt).'));




