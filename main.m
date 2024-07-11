%% 1: Parameters of the system
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
alpha_FLC = 20 * (pi/180);

%% 1: Defining the A, B, C, D matrices for the short period model of the system as well as the state space model

A_sp = [-Z_alpha/V, 1.0; M_alpha, M_q];
B_sp = [-Z_delta/V; M_delta];
C_sp = [-A_alpha/g, 0.0; 0.0, 1];
D_sp = [-A_delta/g; 0.0];

G_m = ss(A_sp, B_sp, C_sp, D_sp);
G_m.InputName = 'u_m';
G_m.StateName = {'x1', 'x2'};
G_m.OutputName = {'y1', 'y2'};
save G_m

%% 1: Defining the A, B, C, D matrices for the actuator model of the system as well as its state space model

A_act = [0.0, 1.0; -omega_a^2, -2*zeta_a*omega_a];
B_act = [0.0; omega_a^2];
C_act = [1.0, 0.0; 0.0, 1.0];
D_act = [0.0; 0.0];

G_act = ss(A_act, B_act, C_act, D_act); 
G_act.InputName = 'u_cmd';
G_act.StateName = {'x3', 'x4'};
G_act.OutputName = {'u_m', 'udot_m'};
save G_act

%% 1: Loading Simulink model to obatin state space of open loop actuator + system dynamics 

sys_am = "Airframe";
load_system(sys_am)
OP_am_object = operspec(sys_am);

% Set the operating point condition
OP_am_object.States(2).Known(1,1) = 1;
OP_am_object.States(2).SteadyState(1,1) = 1;
OP_am_object.States(2).Min(1,1) = -alpha_FLC;
OP_am_object.States(2).Max(1,1) = alpha_FLC;
OP_am_object.States(2).x(1,1) = alpha_FLC;


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

%% 1: Analyze system stability

% Get the transfer function matrix
tf_G_am = tf(G_am_reord);

% Extract individual transfer functions
G_ol_nz = tf_G_am(1);
zpk_G_ol_nz_old = zpk(G_ol_nz);
G_ol_q = tf_G_am(2);
zpk_G_ol_q_old =  zpk(G_ol_q);

% Get the damping information
[wn, zeta, poles] = damp(G_am_reord);
disp('Damping Ratios and Natural Frequencies:');
disp(table(wn, zeta, poles));

% Analyze Acceleration zeros
disp('Zeros of acceleration:');
disp(zero(G_ol_nz));

% Analyze Pitch rate zeros
disp('Zeros of pitch rate:');
disp(zero(G_ol_q));

% Create directory if it does not exist
outputDir = 'Figures';

% figure;
% step(G_m);
% title('Step Response of the Open Loop Missile Model');
% saveas(gcf, fullfile(outputDir, 'StepResponse_MissileModel.pdf'));

% figure;
% step(G_act);
% title('Step Response of the Open Loop Actuator Model');
% saveas(gcf, fullfile(outputDir, 'StepResponse_ActuatorModel.pdf'));

% figure;
% step(G_am_reord, 20);
% title('Step Response of the Open Loop Airframe Model');
% saveas(gcf, fullfile(outputDir, 'StepResponse_AirframeModel.pdf'));

% figure;
% iopzmap(G_am_reord);
% saveas(gcf, fullfile(outputDir, 'ioPZMap_AirframeModel.pdf'));

%% 2: Loop Shaping

% Damping Gain Design

% figure;
% rlocusplot(-G_ol_q);

C_q = -0.163;
% C_q = -0.024;
sys_cq = 'ClosedLoop_Cq';
% Open the Simulink model
load_system(sys_cq);

% Update the Gain block with the value from the workspace
% set_param('ClosedLoop_Cq','Gain','C_q');
% sim("ClosedLoop_Cq.slx");


% % Define the I/O points for linearization
% io_cq(1) = linio('ClosedLoop_Cq/u_unsc', 1, 'input');  % Input at u_unsc
% io_cq(2) = linio('ClosedLoop_Cq/y_1', 1, 'output');    % Output at y1

% Linearize the model
G_cl_q = linearize(sys_cq);
G_cl_q_unsc = G_cl_q(1,1);
% Display the closed-loop transfer function in zpk form
% disp('Closed-Loop Transfer Function G_cl_q_unsc:');
% zpk_G_cl_q_unsc = zpk(G_cl_q_unsc);
% disp(zpk_G_cl_q_unsc);

% Display the open-loop transfer function in zpk form
% disp('Open-Loop Transfer Function G_am(1,1):');
% zpk_G_ol_nz = zpk(G_ol_nz);
% disp(zpk_G_ol_nz);

% figure;
% iopzmap(G_cl_q_unsc, G_ol_nz)
% saveas(gcf, fullfile(outputDir, 'ioPZMap_ClosedLoop_Cq.pdf'));

%iopzmap(G_cl_q_unsc);

% figure;
% step(G_ol_nz);

% Scaling gain Design

k_sc = dcgain(G_cl_q_unsc);

C_sc = 1/k_sc;

sys_cqcs = 'ClosedLoop_CqCsc';
load_system(sys_cqcs);

G = linearize(sys_cqcs);
zpk_G = zpk(G);

figure;
step(G);
% saveas(gcf, fullfile(outputDir, 'Step_Response_G_22.pdf'));

% Integral Gain Design

C_i = 5.48;%5.48

sys_cqcsci = 'ClosedLoop_CqCscCi';
load_system(sys_cqcsci);

G_CqCscCi = linearize(sys_cqcsci);
G_ol_nz_23 = G_CqCscCi(1,1);
zpk_G_ol_nz_23 = zpk(G_ol_nz_23);
% sisotool(G_ol_nz_23);

T_full = linearize(sys_cqcsci);
T = T_full(1,1);
% step(T,[1,10]);
zpk_T = zpk(T);
% saveas(gcf, fullfile(outputDir, 'Step_Response_T_23.pdf'));

%% 3.A/B/C: Mixed Sensitivity (hinfsyn)

% Part #3A - Weighting Filters

% Getting the peak response of W1
M_s_min = 0.5 * (1/sin(15*(pi/180))); %1.93

% Initialize values of makeweight for the inverse of W1
dcgain_w1_dB = -60;
hfgain_w1_db = M_s_min;
mag_w1_dB = -3.01;
freq_w1 = 4;

% Convert dB gains to absolute gains
dcgain_w1_abs = db2mag(dcgain_w1_dB);
mag_w1_abs = db2mag(mag_w1_dB);
hfgain_w1_abs = db2mag(hfgain_w1_db);

W1_inv = makeweight(dcgain_w1_abs, [freq_w1, mag_w1_abs], hfgain_w1_db);


% Initialize values of makeweight for the inverse of W2
dcgain_w2_dB = 100;
hfgain_w2_dB = -60;
mag_w2_dB = -15.3; %-17.5
freq_w2_db = 151; %frequency at -3.01 dB actuator bandwidth

% Convert dB gains to abs gains
hfgain_w2_abs = db2mag(hfgain_w2_dB);
mag_w2_abs = db2mag(mag_w2_dB);
dcgain_w2_abs = db2mag(dcgain_w2_dB);

W2_inv = makeweight(dcgain_w2_abs, [freq_w2_db, mag_w2_abs], hfgain_w2_abs);


% For question 3A.1, first exercise, W3 = W1. Change the mag_w3_dB parameter
% to -3.01
% lines:
% --------------------------------------------
dcgain_w3_dB = -60;
hfgain_w3_db = M_s_min;
mag_w3_dB = -21.5; %-16.50
freq_w3 = 4;

% Convert dB gains to absolute gains
dcgain_w3_abs = db2mag(dcgain_w3_dB);
mag_w3_abs = db2mag(mag_w3_dB);
hfgain_w3_abs = db2mag(hfgain_w3_db);
% 
W3_inv = makeweight(dcgain_w3_abs, [freq_w3, mag_w3_abs], hfgain_w3_abs);
% --------------------------------------------


% Invert W1, W2 and W3 filters to obtain the correct TF
W1  = 1/W1_inv;
W2 = 1/W2_inv;
W3 = 1/W3_inv;

%Plot the singular value magnitude vs frequency of W1 and W1
figure;
sigma(W2_inv, W1_inv);


% Part #3B - Reference Model Computation

% Using fmincon for T_d optimzation -------------------------------
ts_d = 0.18;
Md_d = 5; 

objective = @(x) compute_step_error(x, ts_d, Md_d);

initial_guess_fmincon = [36.6394/2, 0.7]; %Based on Robust Control rule of thumb/usual values of damping ratio

lb = [0, 0];
ub = [inf, 1];

options_fmincon = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

[optimal_params, fval] = fmincon(objective, initial_guess_fmincon, [],[], [], [], lb, ub, [], options_fmincon);

% Extract optimal omega_d and zeta_d from fmincon
omega_d_opt = optimal_params(1); 
zeta_d_opt =  optimal_params(2); 
% --------------------------------------


% Create and plot the step response of the optimized system
T_d_opt = tf([(-omega_d_opt^2/36.6394), omega_d_opt^2], [1, 2*zeta_d_opt*omega_d_opt, omega_d_opt^2]);

figure;
step(T_d_opt);
grid on;
title('Step Response of the Optimized Reference Model');
zpk_T = zpk(T_d_opt);

%Check SettlingTime and Overshoot info

stepinfo(zpk_T, "SettlingTimeThreshold", 0.05)

%Part 3C: Controller Design

%Load Simulink model and linearize its transfer function
sys_3c1 = "Design";
load_system(sys_3c1);

P = linearize(sys_3c1);
zpk_P = zpk(P);

% Obtain Controller transfer function and system Closed Loop transfer
% function through hinfsynthesis

rel_tol = 1*10^-6;
opt_3c = hinfsynOptions( 'Method', 'RIC', 'RelTol', rel_tol);
[C0_e, CL_Twz, gamma] = hinfsyn(P, 1, 1, [0, 10], opt_3c);

% Verifying Hinf synthesis matches the theory
S_o = (1 + G* C0_e)^-1;
T_o = G*C0_e*S_o;
T_wz_theory = [W1*S_o; W2*C0_e*S_o; W3*(T_d_opt - T_o)];

% Defining options for the sigmaplot
p_options = sigmaoptions;
p_options.MagUnits = 'abs';
p_options.FreqScale = 'log';
p_options.Grid = 'on';
p_options.FreqUnits = 'rad/s';

% Display the gamma 
disp('Gamma values of the airframe and weighting filters')
disp(norm(CL_Twz, 'Inf'));
disp(norm(CL_Twz(1), 'Inf'));
disp(norm(CL_Twz(2), 'Inf'));
disp(norm(CL_Twz(3), 'Inf'));

% Singular values plots of the CL weighted Airframe system
figure;
sigmaplot(CL_Twz, p_options);
hold on;
sigmaplot(CL_Twz(1), p_options);
hold on;
sigmaplot(CL_Twz(2), p_options);
hold on;
sigmaplot(CL_Twz(3), p_options);
hold off;
legend('T_wz', 'T_wz1', 'T_wz2', 'T_wz3');

% Part 3.C2:  Controller order reduction

%Calculating the zpk form, the poles and zeros of initial C0_e obtaied from Part 3C.1
zpk_C0_e = zpk(C0_e);
[wn_C0_e, zeta_C0_e, poles_C0_e] = damp(zpk_C0_e);
zeros_C0_e = zero(zpk_C0_e);
wn_zeros_C0_e = abs(zeros_C0_e);

%Displaying the poles and zeros of initial C0_e obtaied from Part 3C.1
disp("These are the poles of C0_e:");
disp(table(wn_C0_e, zeta_C0_e, poles_C0_e, 'VariableNames', {'NaturalFrequency', 'Damping Ratio', 'Poles'}));

disp("These are the zeros of C0_e:");
disp(table(wn_zeros_C0_e, zeros_C0_e, 'VariableNames', {'Zeros', 'Natural Frequency'}));

% Obtaining relevant gains associated with the wanted poles and zeros so as
% to obtain Ce_min 
[Z_C0_e, P_C0_e, K_PZ_C0_e] = zpkdata(C0_e, 'v');


%Iterating through all p/z to identify poles and zeros to be removed:

min_zeros_C0_e = [];
min_poles_C0_e = [];

selected_zeros_C0_e = [];
selected_poles_C0_e = [];

for i = 1:length(Z_C0_e)
    if abs(imag(Z_C0_e(i))) == max(abs(imag(Z_C0_e)))
            min_zeros_C0_e= [min_zeros_C0_e; Z_C0_e(i)];    
    else
            selected_zeros_C0_e = [selected_zeros_C0_e; Z_C0_e(i)];
    end
end           
        

for i = 1:length(P_C0_e)
    if abs(imag(P_C0_e(i))) == max(abs(imag(P_C0_e)))
        min_poles_C0_e = [min_poles_C0_e; P_C0_e(i)];
    else 
        selected_poles_C0_e = [selected_poles_C0_e; P_C0_e(i)];
    end
end

% Finding appropriate gain adjustement

gain_adjust_C0_e_poles = 1/abs(min_poles_C0_e(1));
gain_adjust_C0_e_zeros = abs(min_zeros_C0_e);
gain_scale_C0_e = K_PZ_C0_e * gain_adjust_C0_e_poles *gain_adjust_C0_e_poles*gain_adjust_C0_e_zeros(1)*gain_adjust_C0_e_zeros(2); 
 
% %Creating minimized transfer function C_e_min, displaying it and comparing
% %it to how it was previously
 
C_e_min = zpk(selected_zeros_C0_e, selected_poles_C0_e, gain_scale_C0_e);

disp('The minimized transfer function C_e_min:');
disp(C_e_min);


%Comparison of C0_e and C_e_min

figure;
bode(C0_e, 'r', C_e_min, 'b--'); 
legend('Original C0_e', 'Minimized C_e_{min}');
grid on;
title('Bode Plot Comparison of C0_e and C_e_{min}');


% Obtaining C_i_min by removing the remaning LF pole
[Z_C_e_min, P_C_e_min, K_C_e_min] = zpkdata(C_e_min, 'v');

selected_poles_C_e_min = [];
abs_P_C_e_min = abs(P_C_e_min); 
LF_P_C_e_min = min(abs_P_C_e_min); 

for i = 1:length(P_C_e_min)
    if abs(P_C_e_min(i)) == LF_P_C_e_min
        LF_P_C_e_min = P_C_e_min(i);
    else
        selected_poles_C_e_min = [selected_poles_C_e_min; P_C_e_min(i)];
    end
end

C_i_min = zpk(Z_C_e_min, selected_poles_C_e_min, K_C_e_min);

% Perform model reduction, specifying the oder of the reduced model

% Compute reduced order approximation
R_3c2 = reducespec(C_i_min,'balanced');
% Compute MOR data once
R_3c2 = process(R_3c2);
% Get reduced-order model
Ci_red = getrom(R_3c2,Order=2);

% Compare Bode plots for the original and reduced transfer functions
figure;
bode(C_i_min, Ci_red);
legend('Original Ci_min', 'Reduced Ci_red');
title('Bode Plot Comparison');



% Part 3C.2: Controller Analysis and simulation
F_f = 1;

sys_3c3_CL = 'ClosedLoop_Test';
load_system(sys_3c3_CL)

%Make sure the value of the Ci_red is Ci_red
block_path = [sys_3c3_CL, '/Ci_red'];
set_param(block_path, 'sys', 'Ci_red');

block_path = [sys_3c3_CL, '/F_f'];
set_param(block_path, 'sys', 'F_f');

T_3c3_CL = linearize(sys_3c3_CL);

So_3c3_CL = T_3c3_CL(1,1);
CeSo_3c3_CL = T_3c3_CL(2,1);
To_3c3_CL = T_3c3_CL(3,1);
Tm_3c3_CL = T_3c3_CL(4,1);
T_r_udotm_3c3_CL = T_3c3_CL(6,1);
min_Ti_3c3_CL = T_3c3_CL(2,2);
SoG_3c3_CL = T_3c3_CL(3,2);
Si_3c3_CL = T_3c3_CL(5,2);

% Define the frequency range for singular value plot
omega = logspace(-2.5, 3, 1000);

% Plot the singular values
figure;
subplot(2, 3, 1);
sigma(W1_inv ,'red', So_3c3_CL, 'b', Si_3c3_CL, 'g--', omega);
title('Singular Values of W1^{-1}, S_{o} and S_{i}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W^{-1}', 'S_{o}', 'S_{i}')
grid on;

subplot(2, 3, 2);
sigma(W2_inv, 'r', CeSo_3c3_CL, 'b', omega);
title('Singular Values of W2^{-1} and C_{e}S_{o}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W2^{-1}', 'C_{e}S_{0}')
grid on;

subplot(2, 3, 3);
sigma(W3_inv, 'r', Tm_3c3_CL, 'b', omega);
title('Singular Values of W3^{-1} and T_{o}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W3^{-1}', 'T_{m}')
grid on;

subplot(2, 3, 4);
sigma(-min_Ti_3c3_CL, 'b', To_3c3_CL, 'g--', omega);
title('Singular Values of T_{i} and T_{o}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('T_{i}', 'T_{o}');
grid on;

subplot(2, 3, 5);
sigma(SoG_3c3_CL, 'b', omega);
title('Singular Values of S_{o}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('S_{o}');
grid on;

subplot(2, 3, 6);
sigma(C0_e, 'r', C_e_min, 'g--', omega);
title('Singular Values of C_{0_{e}} and C_{e_{red}}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('C_{0_{e}}', 'C_{e_{red}}')
grid on;

sys_3c3_OL = 'OpenLoop_Test';

load_system(sys_3c3_OL);
block_path = [sys_3c3_OL, '/Ci_red'];
set_param(block_path, 'sys', 'Ci_red');

block_path = [sys_3c3_OL, '/F_f'];
set_param(block_path, 'sys', 'F_f');

T_3c3_OL = linearize(sys_3c3_OL);

% Getting the GM and PM of the open-loop system
[Gm_3c3_OL,Pm_3c3_OL,Wcg_3c3_OL,Wcp_3c3_OL] = margin(T_3c3_OL);

Dm_3c3_OL = (pi/180)*Pm_3c3_OL/Wcg_3c3_OL; %seconds

figure;
bode(T_3c3_OL);
grid on
title('Bode plot of the Open Loop system')

%Third exercise of 3c3
figure;
subplot(2, 2, 1);
step(So_3c3_CL ,'b');
title('Step response of S_{o}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o}')
grid on;

subplot(2, 2, 2);
step(T_d_opt ,'r', To_3c3_CL, 'b');
title('Step response of T_{d} and T_{o}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('T_{d}','T_{o}')
grid on;

subplot(2, 2, 3);
step(SoG_3c3_CL ,'b');
title('Step response of S_{o}G');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o}G')
grid on;

subplot(2, 2, 4);
step((180/pi)*T_r_udotm_3c3_CL ,'b');
title('Step response of T_{r_{udot_{m}}}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('T_{rudotm}');
grid on;


%---------------------------------------------------------
%Save P, its zpk form, C0_e, CL_Twz and associated gamma to structure Results_hinfsyn
Results_hinfsyn.P =P;
Results_hinfsyn.zpk_P = zpk_P;
Results_hinfsyn.C0_e = C0_e;
Results_hinfsyn.CL_Twz = CL_Twz;
Results_hinfsyn.gamma = gamma;
%---------------------------------------------------------
%---------------------------------------------------------
%Save Twz_theory, So and To to structure Results_hinfsyn
Results_hinfsyn.S_o = S_o;
Results_hinfsyn.T_o = T_o;
Results_hinfsyn.T_wz_theory = T_wz_theory;
%---------------------------------------------------------
%---------------------------------------------------------
%Save p_options to structure Results_hinfsyn
Results_hinfsyn.p_options = p_options;
%---------------------------------------------------------
%---------------------------------------------------------
%Save zpk form of C0_e and its poles/zeros to structure Results_hinfsyn
Results_hinfsyn.zpk_C0_e = zpk_C0_e;
Results_hinfsyn.poles_C0_e = poles_C0_e;
Results_hinfsyn.zeros_C0_e = zeros_C0_e;
%---------------------------------------------------------
%---------------------------------------------------------
%Save C_e_min to structure Results_hinfsyn
Results_hinfsyn.C_e_min = C_e_min;
%---------------------------------------------------------
%---------------------------------------------------------
%Save C_i_min and Ci_red to structure Results_hinfsyn
Results_hinfsyn.C_i_min = C_i_min;
Results_hinfsyn.Ci_red = Ci_red;
%---------------------------------------------------------
%---------------------------------------------------------
%Save Ff, Closed Loop T and relavant trasnfer functions to structure Results_hinfsyn
Results_hinfsyn.F_f = F_f;
Results_hinfsyn.T_3c3_CL = T_3c3_CL;
Results_hinfsyn.So_3c3_CL = So_3c3_CL;
Results_hinfsyn.CeSo_3c3_CL = CeSo_3c3_CL;
Results_hinfsyn.To_3c3_CL = To_3c3_CL;
Results_hinfsyn.Tm_3c3_CL = Tm_3c3_CL;
Results_hinfsyn.T_r_udotm_3c3_CL = T_r_udotm_3c3_CL;
Results_hinfsyn.min_Ti_3c3_CL = min_Ti_3c3_CL;
Results_hinfsyn.SoG_3c3_CL = SoG_3c3_CL;
Results_hinfsyn.Si_3c3_CL = Si_3c3_CL;
%---------------------------------------------------------
%---------------------------------------------------------
%Save Open Loop T as well as its relevant margins to structure Results_hinfsyn
Results_hinfsyn.T_3c3_OL = T_3c3_OL;
Results_hinfsyn.Gm_3c3_OL = Gm_3c3_OL;
Results_hinfsyn.Pm_3c3_OL = Pm_3c3_OL;
Results_hinfsyn.Wcg_3c3_OL = Wcg_3c3_OL;
Results_hinfsyn.Wcp_3c3_OL = Wcp_3c3_OL;
Results_hinfsyn.Dm_3c3_OL = Dm_3c3_OL;
%---------------------------------------------------------
%% 3D: Feedback controller design (hinfstruct)

sys_3d1 = "Design";
load_system(sys_3d1);

% Tunable parameters
K_star = 2;
n1_star = 2;
n2_star = 3;
d1_star = 5;
d2_star = 6;

sys_trial = tf([1,n1_star,n2_star],[1,d1_star,d2_star]);

% Define the tunable transfer function with two poles and two zeros
Ce_red_init = tunableTF('Ce_red_star', sys_trial);  % 'Ci_red' has 2 poles and 2 zeros

% Define the integrator
Integrator = tf(K_star, [1 0]);

% Combine the tunable transfer function with the integrator
Ce_red_init = series(Integrator, Ce_red_init);

P_3d1 = linearize(sys_3d1);

% Options for the hinfstruct
RS = 20;
UP = true;
Tol_G = 10^-3;
opt_3d1 = hinfstructOptions('RandomStart', RS, 'UseParallel', UP, 'TolGain', Tol_G);

% Run optimization hinfstruct
[Ce_red_star, gamma_star, info_3d1] = hinfstruct(P_3d1,Ce_red_init,opt_3d1);

Ci_red_star = zpk(Ce_red_star*tf('s'));
Ci_red_star = minreal(Ci_red_star);

figure;
bode(Ci_red_star, Ci_red);




%Get the CL system 
Twz_star = lft(P_3d1, Ce_red_star, 1, 1);

% Display the gammas of the star system
disp('gamma values of the Controller obtained through hinfstruct')
disp(norm(Twz_star, 'Inf'));
disp(norm(Twz_star(1), 'Inf'));
disp(norm(Twz_star(2), 'Inf'));
disp(norm(Twz_star(3), 'Inf'));

omega = logspace(-1, 5, 1000);
% Singular values plots of the 
figure;
sigmaplot(Twz_star, omega, p_options);
hold on;
sigmaplot(Twz_star(1), omega, p_options);
hold on;
sigmaplot(Twz_star(2), omega, p_options);
hold on;
sigmaplot(Twz_star(3), omega, p_options);
hold off;
legend('T_star_wz', 'T_star_wz1', 'T_star_wz2', 'T_star_wz3');

% Bode plot comparing the Ci_red_star to the previous computer controllers
figure;
bode(C_i_min, Ci_red, Ci_red_star);
legend('C_{i_{min}}', 'C{i_{red}}', 'C{i_{redstar}}');
grid on;

%Controller Analysis and simulation --------------------------------------

sys_3d2_CL = 'ClosedLoop_Test';
load_system(sys_3d2_CL);

%Change the value of the Ci_red to Ci_red_star
block_path = [sys_3d2_CL, '/Ci_red'];
set_param(block_path, 'sys', 'Ci_red_star');

T_3d2_CL = linearize(sys_3d2_CL);

So_3d2_CL = T_3d2_CL(1,1);
CeSo_3d2_CL = T_3d2_CL(2,1);
To_3d2_CL = T_3d2_CL(3,1);
Tm_3d2_CL = T_3d2_CL(4,1);
T_r_udotm_3d2_CL = T_3d2_CL(6,1);
min_Ti_3d2_CL = T_3d2_CL(2,2);
SoG_3d2_CL = T_3d2_CL(3,2);
Si_3d2_CL = T_3d2_CL(5,2);

% Define the frequency range for singular value plot
omega = logspace(-2.5, 3, 1000);


% Plot the singular values
figure;
subplot(2, 3, 1);
sigma(W1_inv ,'red', So_3c3_CL, 'b', Si_3c3_CL, 'g--', So_3d2_CL,'magenta', Si_3d2_CL, 'magenta--', omega);
title('Singular Values of W1^{-1}, S_{o}, S_{i}, S_{o}^{*} and S_{i}^{*}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W^{-1}', 'S_{o}', 'S_{i}', 'S_{o}^{*}', 'S_{i}^{*}')
grid on;

subplot(2, 3, 2);
sigma(W2_inv, 'r', CeSo_3c3_CL, 'b', CeSo_3d2_CL, 'magenta', omega);
title('Singular Values of W2^{-1}, C_{e}S_{o}, C_{e}^{*}S_{o}^{*}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W2^{-1}', 'C_{e}S_{0}', 'C_{e}^{*}S_{o}^{*}')
grid on;

subplot(2, 3, 3);
sigma(W3_inv, 'r', Tm_3c3_CL, 'b', Tm_3d2_CL, 'magenta', omega);
title('Singular Values of W3^{-1}, T_{m} and T_{m}^{*}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W3^{-1}', 'T_{m}', 'T_{m}^{*}')
grid on;

subplot(2, 3, 4);
sigma(-min_Ti_3c3_CL, 'b', To_3c3_CL, 'g--', -min_Ti_3d2_CL, 'magenta', To_3d2_CL, 'magenta--', omega);
title('Singular Values of T_{i}, T_{o}, T_{i}^{*}, T_{o}^{*}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('T_{i}', 'T_{o}', 'T_{i}^{*}', 'T_{o}^{*}');
grid on;

subplot(2, 3, 5);
sigma(SoG_3c3_CL, 'b', SoG_3d2_CL, 'magenta', omega);
title('Singular Values of S_{o}G and S_{o}^{*}G');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('S_{o}G', 'S_{o}^{*}G');
grid on;

subplot(2, 3, 6);
sigma(C0_e, 'r', C_e_min, 'g--', Ce_red_star, 'magenta', omega);
title('Singular Values of C_{0_{e}}, C_{e_{red}} and C_{i_{red}}^{*}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('C_{0_{e}}', 'C_{e_{red}}', 'C_{ired}^{*}')
grid on;


% Open the open loop at actuator input
sys_3d2_OL = 'OpenLoop_Test';
load_system(sys_3d2_OL);

%Change the value of the Ci_red to Ci_red_star
block_path = [sys_3d2_OL, '/Ci_red'];
set_param(block_path, 'sys', 'Ci_red_star');

load_system(sys_3d2_OL);
T_3d2_OL = linearize(sys_3d2_OL);

% Getting the GM and PM of the open-loop system
[Gm_3d2_OL,Pm_3d2_OL,Wcg_3d2_OL,Wcp_3d2_OL] = margin(T_3d2_OL);

Dm_3d2_OL = (pi/180)*Pm_3d2_OL/Wcg_3d2_OL; %seconds

figure;
bode(T_3d2_OL);
grid on
title('Bode plot of the Open Loop system')

%Third exercise of 3c3
figure;
subplot(2, 2, 1);
step(So_3c3_CL ,'b', So_3d2_CL, 'magenta');
title('Step response of S_{o} and S_{o}^{*}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o}', 'S_{o}^{*}')
grid on;

subplot(2, 2, 2);
step(T_d_opt ,'r', To_3c3_CL, 'b', To_3d2_CL, 'magenta');
title('Step response of T_{d}, T_{o} and T_{o}^{*}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('T_{d}','T_{o}', 'T_{o}^{*}')
grid on;

subplot(2, 2, 3);
step(SoG_3c3_CL ,'b', SoG_3d2_CL, 'magenta');
title('Step response of S_{o}G and S_{o}^{*}G');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o}G', 'S_{o}^{*}G')
grid on;

subplot(2, 2, 4);
step((180/pi)*T_r_udotm_3c3_CL ,'b', (180/pi)*T_r_udotm_3d2_CL, 'magenta');
title('Step response of T_{r_{udot_{m}}} and T_{r_{udot_{m}}}^{*}');
xlabel('Time[s]');
ylabel('Deflection rate [deg/s]');
legend('T_{rudotm}', 'T_{rudotm}^{*}');
grid on;



Results_hinfstruct = struct();
%---------------------------------------------------------
% Save P, Ce_red_star and its relevant information and Ci_red_star to the structure
Results_hinfstruct.P = P_3d1;
Results_hinfstruct.Ce_red_star = Ce_red_star;
Results_hinfstruct.gamma_star = gamma_star;
Results_hinfstruct.opt_3d1 = opt_3d1;
Results_hinfstruct.info = info_3d1;
Results_hinfstruct.Ci_red_star = Ci_red_star;
%---------------------------------------------------------
%---------------------------------------------------------
% Save Twz_star to the structure
Results_hinfstruct.Twz_star = Twz_star;
%---------------------------------------------------------
Results_hinfstruct.T_3d2_OL = T_3d2_OL;
Results_hinfstruct.Gm_3d2_OL = Gm_3d2_OL;
Results_hinfstruct.Pm_3d2_OL = Pm_3d2_OL;
Results_hinfstruct.Wcg_3d2_OL = Wcg_3d2_OL;
Results_hinfstruct.Wcp_3d2_OL = Wcp_3d2_OL;
Results_hinfstruct.Dm_3d2_OL= Dm_3d2_OL;
%---------------------------------------------------------
%Save Ff, Closed Loop T and relavant trasnfer functions to structure Results_hinfsyn
Results_hinfstruct.F_f = F_f;
Results_hinfstruct.T_3d2_CL = T_3d2_CL;
Results_hinfstruct.So_3d2_CL = So_3d2_CL;
Results_hinfstruct.CeSo_3d2_CL = CeSo_3d2_CL;
Results_hinfstruct.To_3d2_CL = To_3d2_CL;
Results_hinfstruct.Tm_3d2_CL = Tm_3d2_CL;
Results_hinfstruct.T_r_udotm_3d2_CL = T_r_udotm_3d2_CL;
Results_hinfstruct.min_Ti_3d2_CL = min_Ti_3d2_CL;
Results_hinfstruct.SoG_3d2_CL = SoG_3d2_CL;
Results_hinfstruct.Si_3d2_CL = Si_3d2_CL;
%---------------------------------------------------------


%% 3E: Feedforward (ff)

%Part 3E.1 Controller Design

Results_FeedForward = struct();

F_f_init = zpk(T_d_opt * Results_hinfstruct.To_3d2_CL^-1);

Results_FeedForward.F_f_init = F_f_init;

%Part 3E.2 Controller Order Reduction

%Evaluating poles and zeros of F_f_init as well as their relevant
%characteristics 

[wn_F_f_init, damp_F_f_init, poles_F_f_init] = damp(F_f_init);
zeros_F_f_init = zero(F_f_init);
wn_zeros_F_f_init = abs(zeros_F_f_init);

disp('These are pole characteristics of F_f_init');
disp(table(wn_F_f_init, damp_F_f_init, poles_F_f_init));

disp('These are the zero characteritsics of F_f_init');
disp(table(wn_zeros_F_f_init, zeros_F_f_init));

%Removing required poles and zeros to form the truncated controller

[Z_F_f_init, P_F_f_init, K_F_f_init] = zpkdata(F_f_init, 'v');

%Iterating through all P_F_f_init to identify poles and zeros to be truncated:
truncated_zeros_F_f_init = [];
truncated_poles_F_f_init = [];

selected_zeros_F_f_init = [];
selected_poles_F_f_init = [];

for i = 1:length(Z_F_f_init)
    if real(Z_F_f_init(i)) > 0
        truncated_zeros_F_f_init = [truncated_zeros_F_f_init; Z_F_f_init(i)];
    elseif abs(Z_F_f_init(i)) == max(abs(Z_F_f_init))
            truncated_zeros_F_f_init= [truncated_zeros_F_f_init; Z_F_f_init(i)];    
    elseif real(Z_F_f_init(i)) < 0
        selected_zeros_F_f_init = [selected_zeros_F_f_init; Z_F_f_init(i)];
    end
end           
        

for i = 1:length(P_F_f_init)
    if real(P_F_f_init(i)) > 0
        truncated_poles_F_f_init = [truncated_poles_F_f_init; P_F_f_init(i)];
    else 
        selected_poles_F_f_init = [selected_poles_F_f_init; P_F_f_init(i)];
    end
end    

% Finding appropriate gain adjustement

gain_adjust_F_f_init = [];
for i = 1:length(truncated_zeros_F_f_init)
    if abs(truncated_zeros_F_f_init(i)) == max(abs(truncated_zeros_F_f_init))
        gain_adjust_F_f_init = [gain_adjust_F_f_init; abs(truncated_zeros_F_f_init(i))];
    end
end

gain_scale_f_lf = K_F_f_init * gain_adjust_F_f_init(1) *gain_adjust_F_f_init(2); 

%Findfing F_f truncated
F_f_lf = zpk(selected_zeros_F_f_init, selected_poles_F_f_init, gain_scale_f_lf);

if round(dcgain(F_f_lf),3) == round(dcgain(F_f_init),3)
    disp('Yay you did well in scaling');
end

%plotting singular values of both feedforward controllers

figure;
sigma(F_f_init, 'b', F_f_lf, 'r')
title('Singular value plot of F_{f_{init}} vs F_{f_{lf}}')
grid on;
legend('F_{f_{init}}', 'F_{f_{lf}}')

% Get reduced models
reducedFf1 = balred(F_f_lf, 1);
reducedFf2 = balred(F_f_lf, 2);
reducedFf3 = balred(F_f_lf, 3);

% Compare reduce models
figure;
hold on;
bode(F_f_lf, 'r'); % Original system in blue
bode(reducedFf1, 'b'); % 1-state reduction in red
bode(reducedFf2, 'm'); % 2-state reduction in green
bode(reducedFf3, 'g--'); % 3-state reduction in magenta
hold off;
legend('Original (5 states)', 'Reduced (1 state)', 'Reduced (2 states)', 'Reduced (3 states)');
title('Bode Responses of Original and Reduced Models');

%Script below obtained from Model Reducer app 
%-----------------------------------------------------------------
% Reduce LTI model order using balanced truncation 
% Compute reduced order approximation on specified frequency range
R_3e = reducespec(F_f_lf,'balanced');
% Set options for Balanced Truncation specification
% Frequency range for computing state contributions
R_3e.Options.FreqIntervals = [0.1 100];
% Compute MOR data once
R_3e = process(R_3e);
% Get reduced-order model
F_f_3e3 = getrom(R_3e,Order=3);
% Create comparison plot
figure;
bode(F_f_init, F_f_lf, F_f_3e3);
grid on;
title('Bode plot comparison of all three F_{f} controllers');
legend('F_{f_{init}}', 'F_{f_{lf}}', 'F_{f}')
%-----------------------------------------------------------------

%Pole zero map of truncated and reduced

figure;
pzmap(F_f_init, F_f_3e3);
grid on;
legend('F_{f_{init}}', 'F_{f}');
title('PoleZero map of the truncated and reduced F_{f} controllers');

% 3E.3 Controller Analysis and Simulation------------------------------

sys_3e3_CL = 'ClosedLoop_Test';
load_system(sys_3e3_CL);

%Change the value of the F_f to F_f_3e3
block_path = [sys_3e3_CL, '/F_f'];
set_param(block_path, 'sys', 'F_f_3e3');

T_3e3_CL = linearize(sys_3e3_CL);

So_3e3_CL = T_3e3_CL(1,1);
CeSo_3e3_CL = T_3e3_CL(2,1);
Tm_3e3_CL = T_3e3_CL(4,1);
To_3e3_CL = T_3e3_CL(3,1);
T_r_udotm_3e3_CL = T_3e3_CL(6,1);
min_Ti_3e3_CL = T_3e3_CL(2,2);
SoG_3e3_CL = T_3e3_CL(3,2);
Si_3e3_CL = T_3e3_CL(5,2);


figure;
subplot(2, 2, 1);
sigma(W3_inv ,'r', Tm_3c3_CL, 'b', Tm_3d2_CL, 'magenta', Tm_3e3_CL, 'green');
title('Singular Values of W3^{-1}, T_{m}, T_{m}^{*} and T_{m_{ff}}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W3^{-1}', 'T_{m}', 'T_{m}^{*}', 'T_{m_{ff}}')
grid on;

subplot(2, 2, 2);
step(T_d_opt ,'r', To_3c3_CL, 'b', To_3d2_CL, 'magenta', To_3e3_CL, 'green');
title('Step response of T_{d}, T_{o}, T_{o}^{*} and T_{o_{ff}}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('T_{d}','T_{o}', 'T_{o}^{*} ', 'T_{o_{ff}}')
grid on;

subplot(2, 2, 3);
sigma(Ci_red, 'r', Ci_red_star, 'magenta', F_f_3e3, 'green', omega);
title('Singular Values of C_{e_{red}}, C_{i_{red}}^{*} and F_{f}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('C_{e_{red}}', 'C_{ired}^{*}', 'F_{f}')
grid on;

subplot(2, 2, 4);
step((180/pi)*T_r_udotm_3c3_CL ,'b', (180/pi)*T_r_udotm_3d2_CL, 'magenta', (180/pi)*T_r_udotm_3e3_CL, 'green');
title('Step response of T_{r_{udot_{m}}}, T_{r_{udot_{m}}}^{*}, T_{r_{udot_{m_{ff}}}}');
xlabel('Time[s]');
ylabel('Deflection rate [deg/s]');
legend('T_{rudotm}', 'T_{rudotm}^{*}', 'T_{r_{udot_{m_{ff}}}}');
grid on;

% Save relevant variables to the dictionary Results_FeedForward
Results_FeedForward.wn_F_f_init = wn_F_f_init;
Results_FeedForward.F_f_lf = F_f_lf;
Results_FeedForward.F_f_3e3 = F_f_3e3;
Results_FeedForward.T_3e3_CL = T_3e3_CL;
Results_FeedForward.Tm_3e3_CL = Tm_3e3_CL;
Results_FeedForward.To_3e3_CL = To_3e3_CL;
Results_FeedForward.T_r_udotm_3e3_CL = T_r_udotm_3e3_CL;





%% 4: Signal-based redesign (feedback controller systune)

% Initialize values of makeweight for the inverse of W2
dcgain_w2_dB = 100;
hfgain_w2_dB = -60;
mag_w2_dB = -15.3; %-17.5
freq_w2_db = 151; %frequency at -3.01 dB actuator bandwidth

% Convert dB gains to abs gains
hfgain_w2_abs = db2mag(hfgain_w2_dB);
mag_w2_abs = db2mag(mag_w2_dB);
dcgain_w2_abs = db2mag(dcgain_w2_dB);

W2_inv = makeweight(dcgain_w2_abs, [freq_w2_db, mag_w2_abs], hfgain_w2_abs);
W2 = 1/W2_inv;


dcgain_w3_dB = -60;
hfgain_w3_db = M_s_min;
mag_w3_dB = -31; %-24.8
freq_w3 = 4;

% Convert dB gains to absolute gains
dcgain_w3_abs = db2mag(dcgain_w3_dB);
mag_w3_abs = db2mag(mag_w3_dB);
hfgain_w3_abs = db2mag(hfgain_w3_db);
% 
W3_inv = makeweight(dcgain_w3_abs, [freq_w3, mag_w3_abs], hfgain_w3_abs);
W3 =1/W3_inv;

% A. Tuning Interface
% A.1 Tuning Interface generation

% Step 1

Results_systune = struct();
Results_systune.Ga = G_act;
Results_systune.Gm = G_m;
Results_systune.Kq = C_q;
Results_systune.Ksc = C_sc;
Results_systune.Td = T_d_opt;
Results_systune.W1 = W1;
Results_systune.W2 = W2;
Results_systune.W3 = W3;
Results_systune.W1_inv = W1_inv;
Results_systune.W2_inv = W2_inv;
Results_systune.W3_inv = W3_inv;
Results_systune.F_f = 1;
Results_systune.Ci = Ci_red_star;

% Step 2
sys_4A = 'ClosedLoop_Test_systune';
load_system(sys_4A);

block_path = [sys_4A, '/Actuator_(G_a)'];
set_param(block_path, 'sys', 'Results_systune.Ga');

block_path = [sys_4A, '/Missile_(G_m)'];
set_param(block_path, 'sys', 'Results_systune.Gm');

block_path = [sys_4A, '/C_q'];
set_param(block_path, 'sys', 'Results_systune.Kq');

block_path = [sys_4A, '/C_sc'];
set_param(block_path, 'sys', 'Results_systune.Ksc');

block_path = [sys_4A, '/Reference_Model'];
set_param(block_path, 'sys', 'Results_systune.Td');

block_path = [sys_4A, '/Ci_red'];
set_param(block_path, 'sys', 'Results_systune.Ci');

block_path = [sys_4A, '/F_f'];
set_param(block_path, 'sys', 'Results_systune.F_f');

% Step 5

% Create system data with slTuner interface
TunedBlocks = {'ClosedLoop_Test_systune/Ci_red'};
AnalysisPoints = {'ClosedLoop_Test_systune/r/1'; ...
                  'ClosedLoop_Test_systune/Ci_red/1'; ...
                  'ClosedLoop_Test_systune/Demux1/1'; ...
                  'ClosedLoop_Test_systune/Reference_Model/1'; ...
                  'ClosedLoop_Test_systune/Sum1/1'; ...
                  'ClosedLoop_Test_systune/Sum2/1'};
% Specify the custom options
Options = slTunerOptions('AreParamsTunable',false);
% Create the slTuner object
CL0 = slTuner('ClosedLoop_Test_systune',TunedBlocks,AnalysisPoints,Options);

ClosedLoop_Test_systune_Ci_red = tunableTF('ClosedLoop_Test_systune_Ci_red',2,2);
ClosedLoop_Test_systune_Ci_red.Numerator.Value = [19.6439235019643 582.452028248495 8990.92483168058];
ClosedLoop_Test_systune_Ci_red.Denominator.Value = [1 54.116609440897 1308.74650803977];
setBlockParam(CL0,'ClosedLoop_Test_systune/Ci_red',ClosedLoop_Test_systune_Ci_red);

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test_systune/r/1[r]'};
Outputs = {'ClosedLoop_Test_systune/Sum2/1[e1]'};
% Tuning goal specifications
MaxGain = Results_systune.W1_inv; % Maximum gain as a function of frequency
% Create tuning goal for gain
So = TuningGoal.Gain(Inputs,Outputs,MaxGain);
So.Name = 'So'; % Tuning goal name

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test_systune/r/1[r]'};
Outputs = {'ClosedLoop_Test_systune/Ci_red/1[u]'};
% Tuning goal specifications
MaxGain = Results_systune.W2_inv; % Maximum gain as a function of frequency
% Create tuning goal for gain
CeSo = TuningGoal.Gain(Inputs,Outputs,MaxGain);
CeSo.Name = 'CeSo'; % Tuning goal name

% Create tuning goal to limit gain of an I/O transfer function
% Inputs and outputs
Inputs = {'ClosedLoop_Test_systune/r/1[r]'};
Outputs = {'ClosedLoop_Test_systune/Sum1/1[e1_d]'};
% Tuning goal specifications
MaxGain = Results_systune.W3_inv; % Maximum gain as a function of frequency
% Create tuning goal for gain
Tm = TuningGoal.Gain(Inputs,Outputs,MaxGain);
Tm.Name = 'Tm'; % Tuning goal name

% Create option set for systune command
Options = systuneOptions();
Options.RandomStart = 20; % Number of randomized starts
Options.UseParallel = true; % Parallel processing flag

% Set soft and hard goals
SoftGoals = [];
HardGoals = [ So ; ...
              CeSo ; ...
              Tm ];

% Tune the parameters with soft and hard goals
[CL1,fSoft,gHard,Info] = systune(CL0,SoftGoals,HardGoals,Options);

% View tuning results
figure;
viewSpec([SoftGoals;HardGoals],CL1);

% B Feedback Controller redesign(systune)

% B.1 Controller Design
Ci_red_hash = getBlockValue(CL1, 'ClosedLoop_Test_systune/Ci_red');
zpk_Ci_red_hash = zpk(Ci_red_hash);

%Generating PZ Map for Ci_red_star and Ci_red_hash
figure;
grid on;
iopzmap(Results_systune.Ci, zpk_Ci_red_hash);
legend('C_{i,red}^*', 'C_{i,red}^#');

% B.2 Controller Analysis & Simulation
sys_4b2_CL = 'ClosedLoop_Test_systune';
load_system(sys_4b2_CL);

%Change the value of the F_f to be unitary
block_path = [sys_4b2_CL, '/F_f'];
set_param(block_path, 'sys', 'Results_systune.F_f');

%Change the value of the Ci_red to Ci_red_star
block_path = [sys_4b2_CL, '/Ci_red'];
set_param(block_path, 'sys', 'zpk_Ci_red_hash');

T_4b2_CL = linearize(sys_4b2_CL);

So_4b2_CL = T_4b2_CL(1,1);
CeSo_4b2_CL = T_4b2_CL(2,1);
To_4b2_CL = T_4b2_CL(3,1);
Tm_4b2_CL = T_4b2_CL(4,1);
T_r_udotm_4b2_CL = T_4b2_CL(6,1);
min_Ti_4b2_CL = T_4b2_CL(2,2);
SoG_4b2_CL = T_4b2_CL(3,2);
Si_4b2_CL = T_4b2_CL(5,2);

% Plot the singular values
figure;
subplot(2, 3, 1);
sigma(W1_inv ,'red', So_3e3_CL, 'g', So_4b2_CL, 'black', omega);
title('Singular Values of W1^{-1}, S_{o}^{*} and S_{o}^{#}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W^{-1}', 'S_{o}^{*}', 'S_{o}^{#}')
grid on;

subplot(2, 3, 2);
sigma(W2_inv, 'r', CeSo_3e3_CL, 'g', CeSo_4b2_CL, 'black', omega);
title('Singular Values of W2^{-1}, C_{e}S_{o}^{*} and C_{e}S_{o}^{#}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W2^{-1}', 'C_{e}S_{o}^{*}', 'C_{e}S_{o}^{#}')
grid on;

subplot(2, 3, 3);
sigma(W3_inv, 'r', Tm_3e3_CL, 'g', Tm_4b2_CL, 'black', omega);
title('Singular Values of W3^{-1}, T_{m_{ff}} and T_{m}^{#}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('W3^{-1}', 'T_{m_{ff}}', 'T_{m}^{#}')
grid on;

subplot(2, 3, 4);
sigma(-min_Ti_3e3_CL, 'g', To_3e3_CL, 'g--', -min_Ti_4b2_CL, 'black', To_4b2_CL, 'black--', omega);
title('Singular Values of T_{i_{ff}}, T_{o_{ff}}, T_{i}^{#}, T_{i}^{#}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('T_{i_{ff}}', 'T_{o_{ff}}', 'T_{i}^{#}', 'T_{i}^{#}');
grid on;

subplot(2, 3, 5);
sigma(SoG_3e3_CL, 'g', SoG_4b2_CL, 'black', omega);
title('Singular Values of S_{o_{ff}}G and S_{o}^{#}G');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('S_{o_{ff}}G', 'S_{o}^{#}G');
grid on;

subplot(2, 3, 6);
sigma(Ci_red, 'r', Ci_red_star, 'magenta', F_f_3e3, 'green', Ci_red_hash, 'black', omega);
title('Singular Values of C_{e_{red}}, C_{i_{red}}^{*}, F_{f} and C_{i, red}^{#}');
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('C_{e_{red}}', 'C_{ired}^{*}', 'F_{f}', 'C_{i, red}^{#}')
grid on;


% Time-domain analysis

figure;
subplot(2, 2, 1);
step(So_3e3_CL ,'g', So_4b2_CL, 'black');
title('Step response of S_{o}^{*} and S_{o}^{#}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o}^{*}', 'S_{o}^{#}')
grid on;

subplot(2, 2, 2);
step(T_d_opt ,'r', To_3e3_CL, 'g', To_4b2_CL, 'black');
title('Step response of T_{d}, T_{o_{ff}} and T_{o}^{#}');
xlabel('Time[s]');
ylabel('Amplitude');
legend('T_{d}','T_{o_{ff}}', 'T_{o}^{#}')
grid on;

subplot(2, 2, 3);
step(SoG_3e3_CL ,'g', SoG_4b2_CL, 'black');
title('Step response of S_{o_{ff}}G and S_{o}^{#}G');
xlabel('Time[s]');
ylabel('Amplitude');
legend('S_{o_{ff}}G', 'S_{o}^{#}G')
grid on;

subplot(2, 2, 4);
step((180/pi)*T_r_udotm_3e3_CL ,'g', (180/pi)*T_r_udotm_4b2_CL, 'black');
title('Step response of T_{r_{udot_{m_{ff}}}} and T_{r_{udot_{m}}}^{#}');
xlabel('Time[s]');
ylabel('Deflection rate [deg/s]');
legend(' T_{r_{udot_{m_{ff}}}}', 'T_{r_{udot_{m}}}^{#}');
grid on;


%% Function used for fmincon in question 3B.1
function error = compute_step_error(params, ts_d, Md_d)
    omega_d = params(1);
    zeta_d = params(2);

    T_d = tf([(-omega_d^2/36.6394), omega_d^2] , [1, 2*zeta_d*omega_d, omega_d^2]);

    Td_info = stepinfo(T_d, 'SettlingTimeThreshold', 5/100);

    ts_error = (Td_info.SettlingTime - ts_d)^2; %standard L2 norm loss function for optimization
    Md_error = (Td_info.Overshoot - Md_d)^2; %standard L2 norm loss function for optimization

    error = ts_error + Md_error;

end