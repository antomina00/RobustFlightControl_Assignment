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

iopzmap(G_cl_q_unsc);
% figure;
% step(G_cl_q_trial);

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

M_s_min = 0.5 * (1/sin(15*(pi/180))); %1.93

dcgain_w1_dB = 60;
hfgain_w1_db = -M_s_min;
mag_w1_dB = 3.01;
freq_w1 = 4;

% Convert dB gains to absolute gains
dcgain_w1_abs = db2mag(dcgain_w1_dB);
mag_w1_abs = db2mag(mag_w1_dB);
hfgain_w1_abs = db2mag(hfgain_w1_db);

W1 = makeweight(dcgain_w1_abs, [freq_w1, mag_w1_abs], hfgain_w1_abs);
% W3 = W1;
% figure;
% sigma(1/W1);


dcgain_w2_dB = -100;
hfgain_w2_dB = 40;
mag_w2_dB = 15;
freq_w2_db = 151;

% Convert dB gains to abs gains
hfgain_w2_abs = db2mag(hfgain_w2_dB);
mag_w2_abs = db2mag(mag_w2_dB);
% freq_w2_abs = 10^(freq_w2_db / 20);
dcgain_w2_abs = db2mag(dcgain_w2_dB);

W2 = makeweight(dcgain_w2_abs, [freq_w2_db, mag_w2_abs], hfgain_w2_abs);

dcgain_w3_dB = -60;
hfgain_w3_db = M_s_min;
mag_w3_dB = -40;
freq_w3 = 4;

% Convert dB gains to absolute gains
dcgain_w3_abs = db2mag(dcgain_w3_dB);
mag_w3_abs = db2mag(mag_w3_dB);
hfgain_w3_abs = db2mag(hfgain_w3_db);

W3 = makeweight(dcgain_w3_abs, [freq_w3, mag_w3_abs], hfgain_w3_abs);

% sigma(1/W3);

% figure;
% bodemag(W2);
% 
W1_inv  = 1/W1;
W2_inv = 1/W2;
% figure;
% sigma(W1_inv, W2_inv);

% W1_trial = tf([1/dcgain_w1_abs, freq_w1] , [1, freq_w1 * M_s_min]);
% sigma(1/W1_trial);

% Part #3B - Reference Model Computation
ts_d = 0.18;
Md_d = 5; 

objective = @(x) compute_step_error(x, ts_d, Md_d);

initial_guess = [36.6394*0.5, 0.1];

lb = [0,0];
ub = [inf, 1];

options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% [optimal_params, fval] = fmincon(objective, initial_guess, [],[], [], [], lb, ub, [], options);

% Extract optimal omega_d and zeta_d
% % omega_d_opt = optimal_params(1);
% % zeta_d_opt = optimal_params(2);

omega_d_opt = 30;%22.6
zeta_d_opt = 0.74;%0.79
counter = 0;

for i = 0:0.1:30
    for j = 0.6:0.001:0.85
        counter = counter;
        fprintf('Iteration: %.4f\n', counter);
        T_d = tf([(-i^2/36.6394), i^2], [1, 2*j*i, i^2]);
        T_d_stepinfo = stepinfo(T_d);
        if T_d_stepinfo.SettlingTime <=0.18 && T_d_stepinfo.Overshoot <=5
            omega_d_trial = i;
            fprintf('Optimal omega_d: %.4f\n', omega_d_trial);
            zeta_d_trial = j;
            fprintf('Optimal zeta_d: %.4f\n', zeta_d_trial);
        else 
            disp('No omega and zeta found')
        end
        counter = counter + 1;
    end
end


% Display the results
% fprintf('Optimal omega_d: %.4f\n', omega_d_opt);
% fprintf('Optimal zeta_d: %.4f\n', zeta_d_opt);

% Plot the step response of the optimized system
T_d_opt = tf([(-omega_d_opt^2/36.6394), omega_d_opt^2], [1, 2*zeta_d_opt*omega_d_opt, omega_d_opt^2]);
% step(T_d_opt);
% grid on;
% title('Step Response of the Optimized Reference Model');
figure;
step(T_d_opt);
zpk_T = zpk(T_d_opt);

%Part 3C: Controller Design

sys_3c1 = "Design";
open_system(sys_3c1);

P = linearize(sys_3c1);
zpk_P = zpk(P);
rel_tol = 1*10^-6;

opt_3c = hinfsynOptions( 'Method', 'RIC', 'RelTol', rel_tol);
[C_e, CL_Tzw, gamma] = hinfsyn(P, 1, 1, [0, 10], opt_3c);

S_o = (1 + G* C_e)^-1;
T_o = G*C_e*S_o;

T_wz_theory = [W1*S_o; W2*C_e*S_o; W3*(T_d_opt - T_o)];

p_options = sigmaoptions;

p_options.MagUnits = 'abs';
p_options.FreqScale = 'log';
p_options.Grid = 'on';
p_options.FreqUnits = 'rad/s';
% p_options.XLimMode = 'manual';
% p_options.XLim = {[0.1, 10^8]};

% figure;
% sigmaplot(CL_Tzw, p_options);
% hold on;
% sigmaplot(CL_Tzw(1), p_options);
% hold on;
% sigmaplot(CL_Tzw(2), p_options);
% hold on;
% sigmaplot(CL_Tzw(3), p_options);
% hold off;
% legend('T_wz', 'T_wz1', 'T_wz2', 'T_wz3');
% 


function error = compute_step_error(params, ts_d, Md_d)
    omega_d = params(1);
    zeta_d = params(2);

    T_d = tf([(-omega_d^2/36.6394), omega_d^2] , [1, 2*zeta_d*omega_d, omega_d^2]);

    Td_info = stepinfo(T_d);

    ts_error = 0.5*(Td_info.SettlingTime - ts_d)^2;
    Md_error = 0.5*(Td_info.Overshoot - Md_d)^2;

    error = ts_error + Md_error;

end


