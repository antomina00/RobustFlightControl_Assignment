%% Parameters of the system
g = 9.80665; % m/s^2 gravity
a = 316.0561; %m/s at operating point altitude 6096m
M = 3; % Mach number at operating point
Z_alpha = 1236.8918; % m/s^2 normal force derivative
M_alpha = -300.4211 ; %1/s^2 pitch moment derivative
M_q = 0.0; %1/s damping derivative
Z_delta = 108.1144; % m/s^2 control force derivative
M_delta = -131.3944; %1/s^2 control moment derivative
A_alpha  = 1434.7783; % m/s^2*rad normal acc derivative
A_delta = 115.0529; %m/s^2*rad control acc derivative
omega_a = 150; % rad/s actuator nat freq
zeta_a = 0.7; % actuator damping ratio	 
V = M*a; % m/s velocity at operating point

%% Defining the A, B, C, D matrices for the short period model of the system as well as the state space model

A_sp = [-Z_alpha/V, 1.0; M_alpha, M_q];
B_sp = [-Z_delta/V; M_delta];
C_sp = [-A_alpha/g, 0.0; 0.0, 1];
D_sp = [-A_delta/g; 0.0];

G_m = ss(A_sp, B_sp, C_sp, D_sp);
G_m.InputName = 'u_m';
G_m.StateName = {'x1', 'x2'};
G_m.OutputName = {'y1', 'y2'};
save G_m

%% Defining the A, B, C, D matrices for the actuator model of the system as well as its state space model

A_act = [0.0, 1.0; -omega_a^2, -2*zeta_a*omega_a];
B_act = [0.0; omega_a^2];
C_act = [1.0, 0.0; 0.0, 1.0];
D_act = [0.0; 0.0];

G_act = ss(A_act, B_act, C_act, D_act); 
G_act.InputName = 'u_cmd';
G_act.StateName = {'x3', 'x4'};
G_act.OutputName = {'u_m', 'udot_m'};
save G_act

%% Loading Simulink model to obatin state space of open loop actuator + system dynamics 

sys_am = "Airframe";
open_system(sys_am)
OP_am_object = operspec(sys_am);
options = findopOptions;
options.OptimizerType = 'graddescent';
options.DisplayReport = 'on';
[OP_am, OpPointReport] = findop(sys_am, OP_am_object, options);

io = getlinio(sys_am);
[G_am, LinPoint, LinModelReport] = linearize(sys_am, io, OP_am);
T_reorder = [0,0,1,0;
             0,0,0,1;
             1,0,0,0;
             0,1,0,0];
G_am_reord = ss2ss(G_am, T_reorder);





