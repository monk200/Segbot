g = 9.81;       % gravity [m/sec^2]
m = 0.062;      % wheel weight [kg]
R = 0.0325;     % wheel radius [m]
J_w = (m*R^2)/2;        % wheel inertia moment [kgm^2]
M = 2.245;      % body weight [kg]
L = 11.86;    % distance of center of mass from the wheel axis [m]
J_psi = (M*L^2)/3;      % body pitch inertia moment [kgm^2]
J_m = 5.432e-11;        % DC motor inertial moment [kgm^2]
R_m = 6.667;        % DC motor resistance [Ω]
K_b = 0.4903;       % DC motor back EMF constant [Vsec/rad]
K_t = 0.4903;       % DC motor torque constant [Nm/A]
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

A = [0 1 0 0;
    Dib(1,:);
    0 0 0 1;
    Dib(2,:)]
B = [0;
    Dih(1);
    0;
    Dih(2)]
A3 = [A(1,1) A(1,2) A(1,4);
    A(2,1) A(2,2) A(2,4);
    A(4,1) A(4,2) A(4,4)]
B3 = [B(1);
    B(2);
    B(4)]
K3 = [-8.7484  -13.7866   -6.6363]; %Bad set of gains to start
K = [-151.7589  -14.8192  -61.2183   -7.1023];  %Bad set of gains to start
