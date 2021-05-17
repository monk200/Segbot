g = 9.81;       % gravity [m/sec^2]
m = 0.0615;     % wheel weight [kg]
R = 0.067;      % wheel radius [m]
J_w = (m*R^2)/2;        % wheel inertia moment [kgm^2]
M = 0.5764;     % body weight [kg]
L = 0.049;      % body width [m]
J_psi = (M*L^2)/3;      % body pitch inertia moment [kgm^2]
J_m = 2.285e-4;     % DC motor inertial moment [kgm^2]
R_m = 1.875;        % DC motor resistance [Ω]
K_b = 0.30646;      % DC motor back EMF constant [Vsec/red]
K_t = 0.30646;      % DC motor torque constant [Nm/A]
n = 1;      % gear ratio
f_m = 0.0022;       % friction coefficient between body and DC motor
alpha = (n*K_t)/R_m;
beta = ((n*K_t*K_b)/R_m) + f_m;

D = [(M*L^2 + J_psi + 2*n^2*J_m) (M*L*R - 2*n^2*J_m);
    (M*L*R - 2*n^2*J_m) ((2*m+M)*R^2 + 2*J_w + 2*n^2*J_m)];
b = [M*g*L -2*beta 0 2*beta;
    0 2*beta 0 -2*beta];
h = [-2*alpha;
    2*alpha];
Dib = inv(D)*b;
Dih = inv(D)*h;

A4 = [0 1 0 0;
    Dib(1,:);
    0 0 0 1;
    Dib(2,:)];
B4 = [0;
    Dih(1);
    0;
    Dih(2)];
A = [A4(1,1) A4(1,2) A4(1,4);
    A4(2,1) A4(2,2) A4(2,4);
    A4(4,1) A4(4,2) A4(4,4)]
B = [B4(1);
    B4(2);
    B4(4)]
K = [-30,-2.8,-1]*6/10  % 6/10 because Simulation max is 6V and real system max is 10 for pwm output
