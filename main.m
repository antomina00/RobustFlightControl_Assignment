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
alpha_FLC = 20 * (pi/180);

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

%% Analyze system stability

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

%% Loop Shaping

% Damping Gain Design

% figure;
% rlocusplot(-G_ol_q);

C_q = -0.163;
% C_q = -0.024;
sys_cq = 'ClosedLoop_Cq';
% Open the Simulink model
open_system(sys_cq);

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
open_system(sys_cqcs);

G = linearize(sys_cqcs);
zpk_G = zpk(G);

% figure;
% step(G);
% saveas(gcf, fullfile(outputDir, 'Step_Response_G_22.pdf'));

% Integral Gain Design

C_i = 5.48;%5.48

sys_cqcsci = 'ClosedLoop_CqCscCi';
open_system(sys_cqcsci);

G_CqCscCi = linearize(sys_cqcsci);
G_ol_nz_23 = G_CqCscCi(1,1);
zpk_G_ol_nz_23 = zpk(G_ol_nz_23);
% sisotool(G_ol_nz_23);

T_full = linearize(sys_cqcsci);
T = T_full(1,1);
% step(T,[1,10]);
zpk_T = zpk(T);
% saveas(gcf, fullfile(outputDir, 'Step_Response_T_23.pdf'));

%% Mixed Sensitivity

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

% W3_inv = W1_inv;

% Initialize values of makeweight for the inverse of W2
dcgain_w2_dB = 100;
hfgain_w2_dB = -40;
mag_w2_dB = -15;
freq_w2_db = 151;

% Convert dB gains to abs gains
hfgain_w2_abs = db2mag(hfgain_w2_dB);
mag_w2_abs = db2mag(mag_w2_dB);
dcgain_w2_abs = db2mag(dcgain_w2_dB);

W2_inv = makeweight(dcgain_w2_abs, [freq_w2_db, mag_w2_abs], hfgain_w2_abs);

% For question 3C.1, first exercise, W3 != W1. Uncomment the following
% lines:
% --------------------------------------------

dcgain_w3_dB = -60;
hfgain_w3_db = M_s_min;
mag_w3_dB = -16.2;
freq_w3 = 4;

% Convert dB gains to absolute gains
dcgain_w3_abs = db2mag(dcgain_w3_dB);
mag_w3_abs = db2mag(mag_w3_dB);
hfgain_w3_abs = db2mag(hfgain_w3_db);

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

% Trying to use fmincon -------------------------------
% ts_d = 0.18;
% Md_d = 5; 
% 
% objective = @(x) compute_step_error(x, ts_d, Md_d);
% 
% initial_guess = [36.6394*0.5, 0.1];
% 
% lb = [0,0];
% ub = [inf, 1];
% 
% options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% [optimal_params, fval] = fmincon(objective, initial_guess, [],[], [], [], lb, ub, [], options);

% Extract optimal omega_d and zeta_d
% % omega_d_opt = optimal_params(1); %37.4518
% % zeta_d_opt = optimal_params(2); %0.716

% --------------------------------------

% Reference model values obtained through trial and error
omega_d_opt = 22.6;
zeta_d_opt = 0.79;
counter = 0;

% Plot the step response of the optimized system
T_d_opt = tf([(-omega_d_opt^2/36.6394), omega_d_opt^2], [1, 2*zeta_d_opt*omega_d_opt, omega_d_opt^2]);

figure;
step(T_d_opt);
grid on;
title('Step Response of the Optimized Reference Model');
zpk_T = zpk(T_d_opt);


%Part 3C: Controller Design

sys_3c1 = "Design";
open_system(sys_3c1);

P = linearize(sys_3c1);
zpk_P = zpk(P);

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
disp('gamma values of the airframe and weighting filters')
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
nat_freq_zeros_C0_e = sqrt(real(zeros_C0_e).^2 + imag(zeros_C0_e).^2);

%Displaying the poles and zeros of initial C0_e obtaied from Part 3C.1
disp("These are the poles of C0_e:");
disp(table(wn_C0_e, zeta_C0_e, poles_C0_e, 'VariableNames', {'NaturalFrequency', 'Damping Ratio', 'Poles'}));

disp("These are the zeros of C0_e:");
disp(table(zeros_C0_e, nat_freq_zeros_C0_e, 'VariableNames', {'Zeros', 'Natural Frequency'}));

% Obtaining relevant gains associated with the wanted poles and zeros so as
% to obtain Ce_min 
[Z_C0_e, P_C0_e, K_PZ_C0_e] = zpkdata(C0_e, 'v');

selected_zeros_C0_e = [zeros_C0_e(1); zeros_C0_e(4:8)]; % %zeros_C0_e(1); zeros_C0_e(3:8)
selected_poles_C0_e = [poles_C0_e(1:6); poles_C0_e(9)]; % poles_C0_e(9)

% % Calculating the gain adjustement due to very HF pole and zero
hf_pole_C0_e = poles_C0_e(7:8);
hf_zero_C0_e = zeros_C0_e(2:3); %2:3 1:2
gain_adjustment_C0_e_1 = abs(hf_zero_C0_e(1));
gain_adjustment_C0_e_2 = abs(hf_zero_C0_e(2));
gain_adjustment_C0_e_3 = 1/abs(hf_pole_C0_e(1));
gain_adjustment_C0_e_4 = 1/abs(hf_pole_C0_e(2));
% gain_adjustment_C0_e_5 = 1/abs(hf_pole_C0_e(3));
% gain_adjustment_C0_e_6 = abs(hf_zero_C0_e(3));

% % Applying adjustement to gain found from zpkdata
K_C0_e_min = K_PZ_C0_e * gain_adjustment_C0_e_1*gain_adjustment_C0_e_2*gain_adjustment_C0_e_3*gain_adjustment_C0_e_4;
% 
% %Creating minimized transfer function C_e_min, displaying it and comparing
% %it to how it was previously
% 
C_e_min = zpk(selected_zeros_C0_e, selected_poles_C0_e, K_C0_e_min);
% 
disp('The minimized transfer function C_e_min:');
disp(C_e_min);
% 
figure;
bode(C0_e, 'r', C_e_min, 'b--');%  
legend('Original C0_e', 'Minimized C_e_{min}');%
grid on;
title('Bode Plot Comparison of C0_e and C_e_{min}');

% Obtaining C_i_min by removing the remaning LF pole 0.005264
selected_poles_C_i_min = [poles_C0_e(2:6); poles_C0_e(9)];
C_i_min = zpk(selected_zeros_C0_e, selected_poles_C_i_min, K_C0_e_min);

% Perform model reduction, specifying the oder of the reduced model
Ci_red = balred(minreal(C_i_min),2); 

% Compare Bode plots for the original and reduced transfer functions
figure;
bode(C_i_min, Ci_red);
legend('Original Ci_min', 'Reduced Ci_red');
title('Bode Plot Comparison');

figure;
pzmap(C_i_min, Ci_red);
legend('C_{i_{min}}', 'C{i_{red}}');
grid on;

[mag_C_i_min, phase_C_i_min, wn_C_i_min] = bode(C_i_min);
[mag_C_i_red, phase_C_i_red, wn_C_i_red] = bode(Ci_red);
phase_peak_C_i_min = max(phase_C_i_min);
phase_peak_Ci_red = max(phase_C_i_red);
% Part 3C.2: Controller Analysis and simulation
F_f = 1;

sys_3c3_CL = 'ClosedLoop_Test';
open_system(sys_3c3_CL);
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
omega = logspace(-3, 3, 1000);

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
open_system(sys_3c3_OL);
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
ylabel('Magnitude');
legend('S_{o}')
grid on;

subplot(2, 2, 2);
step(T_d_opt ,'r', To_3c3_CL, 'b');
title('Step response of T_{d} and T_{o}');
xlabel('Time[s]');
ylabel('Magnitude');
legend('T_{d}','T_{o}')
grid on;

subplot(2, 2, 3);
step(SoG_3c3_CL ,'b');
title('Step response of S_{o}G');
xlabel('Time[s]');
ylabel('Magnitude');
legend('S_{o}G')
grid on;

subplot(2, 2, 4);
step((180/pi)*T_r_udotm_3c3_CL ,'b');
title('Step response of T_{r_{\dot{u}_{m}}}');
xlabel('Time[s]');
ylabel('Magnitude');
legend('T_{r_{\dot{u}_{m}}}')
grid on;

% Function used for fmincon in question 3B.1
% function error = compute_step_error(params, ts_d, Md_d)
%     omega_d = params(1);
%     zeta_d = params(2);
% 
%     T_d = tf([(-omega_d^2/36.6394), omega_d^2] , [1, 2*zeta_d*omega_d, omega_d^2]);
% 
%     Td_info = stepinfo(T_d);
% 
%     ts_error = 0.5*(Td_info.SettlingTime - ts_d)^2;
%     Md_error = 0.5*(Td_info.Overshoot - Md_d)^2;
% 
%     error = ts_error + Md_error;
% 
% end


