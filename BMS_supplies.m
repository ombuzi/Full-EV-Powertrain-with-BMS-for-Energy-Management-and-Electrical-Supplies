% Full EV Powertrain with BMS for Energy Management and Electrical Supplies
% Author: Frank Otieno
% Date: August 09, 2025
% Models full 600km EV with HV/LV supplies, gearbox, eCVT, auxiliaries

clear; clc; close all;

%% Section 1: Parameter Initialization
% Battery Pack
NomCap = 100; % Ah
NomV = 400; % V
R0 = 0.01; R1 = 0.005; R2 = 0.01;
C1 = 1000; C2 = 5000;
Tau1 = R1*C1; Tau2 = R2*C2;
OCV_func = @(SoC) interp1(0:0.1:1, 3.2 + 0.8*(0:0.1:1) + 0.2*(0:0.1:1).^2 - 0.1*(0:0.1:1).^3, SoC, 'spline') * (NomV / 3.7);
SoC_init = 1; T_amb = 25; Alpha_T = 0.005; h_conv = 10; A_pack = 2; mC = 1000;
T_ref = 30; u_max = 100; MPC_horizon = 10; w1 = 1; w2 = 0.1;
NumGroups = 4; SoC_groups_init = [0.98, 0.99, 1.00, 1.01]; Bal_eff = 0.9;

% Vehicle/Powertrain
m_veh = 2000; g = 9.81; Cr = 0.01; CdA = 0.6; rho = 1.2; rw = 0.3;
gear_ratios = [8, 4]; % Low/high
eCVT_min = 4; eCVT_max = 12;
eta_motor = 0.95; eta_regen = 0.9; eta_gear = 0.96;
P_motor_front = 150e3; P_motor_rear = 200e3; % W

% Electrical Supplies
eta_inv = 0.97; f_sw = 10e3; E_onoff = 1e-3; R_cond = 0.001; % Inverter
eta_dc = 0.95; V_lv_init = 12; P_aux_base = 2e3; % DC-DC and auxiliaries (kW)

% Sim Settings
dt = 1; t_end = 21600; % 6 hours for 600km equiv
t = 0:dt:t_end; N = length(t);

% Drive Cycle: Scaled WLTP + highway + hill
v = 27.8 + 10*sin(2*pi*t/1800) + 30*(t>5400 & t<16200) + 40*exp(-((t-18000)/600).^2); % m/s avg 100km/h
grade = 0.05*sin(2*pi*t/3600); % rad

%% Section 2: Full Vehicle Simulation Loop
% States: Battery [SoC, V_RC1, V_RC2, T, SoH, groups], HV_bus V, LV_bus V, P_mot_f, P_mot_r, P_aux
x_bat = zeros(3+1+1+NumGroups, N); x_bat(:,1) = [SoC_init; 0; 0; T_amb; 1; SoC_groups_init'];
V_hv = NomV * ones(N,1); V_lv = V_lv_init * ones(N,1);
P_mot_f = zeros(N,1); P_mot_r = zeros(N,1); P_aux = P_aux_base * ones(N,1);
I_bat = zeros(N,1); P_regen = zeros(N,1); u_cool = zeros(N,1);
gear_idx = ones(N,1); eCVT_ratio = eCVT_min * ones(N,1);
P_loss_inv = zeros(N,1); P_dc = zeros(N,1);
eta_trans_arr = zeros(N,1); P_mot_tot = zeros(N,1);

for k = 2:N
    SoC_k = x_bat(1,k-1); V_RC1_k = x_bat(2,k-1); V_RC2_k = x_bat(3,k-1);
    T_k = x_bat(4,k-1); SoH_k = x_bat(5,k-1); SoC_groups_k = x_bat(6:end,k-1);
    
    % Adjust resistances
    R0_adj = R0 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R1_adj = R1 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    R2_adj = R2 * (1 + Alpha_T*(T_k - 25)) / SoH_k;
    
    % Vehicle Dynamics: Force demand
    a_k = (v(k) - v(k-1)) / dt; % Accel
    Drag = 0.5 * CdA * rho * v(k-1)^2;
    Roll = Cr * m_veh * g;
    Grade = m_veh * g * sin(grade(k-1));
    F_req = m_veh * a_k + Drag + Roll + Grade;
    tau_req = F_req * rw;
    
    % Gear/eCVT: Update based on v/SoC
    gear_idx(k) = 1 + (v(k-1) > 20); % Shift at 20 m/s
    eCVT_ratio(k) = eCVT_min + (eCVT_max - eCVT_min) * (1 - SoC_k); % Higher if low SoC
    eta_trans = eta_gear * (0.98 - 0.02*(eCVT_ratio(k) - eCVT_min)/(eCVT_max - eCVT_min));
    eta_trans_arr(k) = eta_trans;
    
    % Torque Vectoring: 40/60 front/rear base, adjust for stability
    split_front = 0.4 + 0.1 * sin(grade(k-1)); % More front on uphill
    tau_f = split_front * tau_req / gear_ratios(gear_idx(k)) / eCVT_ratio(k);
    tau_r = (1 - split_front) * tau_req / gear_ratios(gear_idx(k)) / eCVT_ratio(k);
    
    % Motor Powers
    omega = v(k-1) / rw;
    P_mot_f(k) = tau_f * omega / eta_motor;
    P_mot_r(k) = tau_r * omega / eta_motor;
    P_mot_tot(k) = P_mot_f(k) + P_mot_r(k);
    
    % Regen if negative
    if P_mot_tot(k) < 0
        P_regen(k) = -P_mot_tot(k) * eta_regen * eta_trans;
    end
    
    % Electrical Supplies: Inverters, DC-DC
    I_inv_f = P_mot_f(k) / V_hv(k-1); I_inv_r = P_mot_r(k) / V_hv(k-1);
    P_loss_inv(k) = f_sw * E_onoff * 2 + (I_inv_f^2 + I_inv_r^2) * R_cond; % Switching + conduction
    P_inv_f = P_mot_f(k) / eta_inv + P_loss_inv(k)/2;
    P_inv_r = P_mot_r(k) / eta_inv + P_loss_inv(k)/2;
    P_aux(k) = P_aux_base + 0.01 * v(k-1) + 0.05 * (T_k - 25); % Variable aux
    P_dc(k) = P_aux(k) / eta_dc;
    P_bat = P_inv_f + P_inv_r + P_dc(k); % Total draw
    
    % BMS Limits: SoP based
    V_ocv = OCV_func(SoC_k);
    SoP_dis = (V_ocv - 300) * V_ocv / R0_adj;
    SoP_chg = (450 - V_ocv) * V_ocv / R0_adj;
    P_bat = min(max(P_bat, -SoP_chg), SoP_dis);
    
    % Battery Current
    I_k = P_bat / V_hv(k-1);
    I_bat(k) = I_k;
    
    % ECM/Thermal/Balancing Update
    dV_RC1 = -V_RC1_k / Tau1 + I_k / C1;
    dV_RC2 = -V_RC2_k / Tau2 + I_k / C2;
    V_RC1_new = V_RC1_k + dt * dV_RC1;
    V_RC2_new = V_RC2_k + dt * dV_RC2;
    dSoC = -I_k * dt / (3600 * NomCap * SoH_k);
    SoC_new = SoC_k + 0.98 * dSoC;
    Q_gen = I_k^2 * R0_adj + (V_RC1_k^2 / R1_adj) + (V_RC2_k^2 / R2_adj);
    u_opt = quadprog(2*(w1*eye(MPC_horizon)+w2*eye(MPC_horizon)), -2*w1*T_ref*ones(MPC_horizon,1), [], [], [], [], zeros(MPC_horizon,1), u_max*ones(MPC_horizon,1), [], optimoptions('quadprog','Display','off'));
    u_cool(k) = u_opt(1);
    Q_cool = u_cool(k) * h_conv * A_pack * (T_k - T_amb);
    dT = (Q_gen - Q_cool) / mC;
    T_new = T_k + dt * dT;
    Delta_SoC = SoC_groups_k - mean(SoC_groups_k);
    transfer = -Delta_SoC * (NomCap/NumGroups) * NomV / Bal_eff;
    SoC_groups_new = SoC_groups_k - transfer / ((NomCap/NumGroups) * NomV) + dSoC;
    dSoH = -0.0001 * abs(dSoC); SoH_new = SoH_k + dSoH;
    
    % Update Bus Voltages (simple droop model)
    V_hv(k) = V_hv(k-1) - I_k * 0.001; % Capacitor droop
    V_lv(k) = V_lv(k-1) * (1 - P_dc(k) / 1e5); % LV stability
    
    % Store
    x_bat(:,k) = [SoC_new; V_RC1_new; V_RC2_new; T_new; SoH_new; SoC_groups_new];
end

%% Section 3: Results Visualization (Multiple Graphs)
% HV/LV Bus Voltages
figure(1);
yyaxis left; plot(t/3600, V_hv, 'b-'); ylabel('HV Bus Voltage (V)');
yyaxis right; plot(t/3600, V_lv, 'r--'); ylabel('LV Bus Voltage (V)');
xlabel('Time (hours)');
title('Electrical Bus Voltages');
grid on;

% Power Splits
figure(2);
area(t/3600, [P_mot_f P_mot_r P_aux abs(P_regen)]/1e3); % Stacked, abs for regen
xlabel('Time (hours)'); ylabel('Power (kW)');
legend('Front Motor', 'Rear Motor', 'Auxiliaries', 'Regen');
title('Power Distribution in Supplies');
grid on;

% Efficiency Maps (Scatter for trans vs v)
figure(3);
scatter(v, eta_trans_arr, 20, P_mot_tot/1e3, 'filled');
xlabel('Velocity (m/s)'); ylabel('Transmission Efficiency');
colorbar; title('Efficiency Map vs Velocity and Power');
grid on;

% Thermal Profile
figure(4);
plot(t/3600, x_bat(4,:), 'g-');
xlabel('Time (hours)'); ylabel('Temperature (C)');
title('Battery Thermal Dynamics');
grid on;

% SoC and Range Estimate
cum_dist = cumsum(v * dt / 3600); % km
range_est = cum_dist(end) / (1 - x_bat(1,end)) * (1 - 0.2); % Project to 20% SoC
figure(5);
yyaxis left; plot(t/3600, x_bat(1,:), 'b-'); ylabel('SoC');
yyaxis right; plot(t/3600, cum_dist, 'c-'); ylabel('Distance (km)');
xlabel('Time (hours)');
title(sprintf('SoC and Cumulative Distance (Range Est: %.1f km)', range_est));
grid on;

% Regen Energy
figure(6);
plot(t/3600, cumsum(P_regen * dt / 3600 / 1000), 'c-');
xlabel('Time (hours)'); ylabel('Cumulative Regen (kWh)');
title('Regenerative Energy Recovery');
grid on;

% Auxiliary Load Variation
figure(7);
scatter(v, P_aux/1e3, 20, x_bat(4,:), 'filled');
xlabel('Velocity (m/s)'); ylabel('Aux Power (kW)');
colorbar; title('Auxiliary Load vs Velocity and Temp');
grid on;

% Energy Flow (Bar for average)
avg_p = [mean(P_mot_f), mean(P_mot_r), mean(P_aux), mean(abs(P_regen)), mean(P_loss_inv)] / 1e3;
figure(8);
bar(avg_p);
xticklabels({'Front Motor', 'Rear Motor', 'Aux', 'Regen', 'Inv Loss'});
ylabel('Average Power (kW)');
title('Average Energy Flow');
grid on;

%% Section 4: Analysis
Energy_regen = trapz(t, P_regen) / 3600 / 1000; % kWh
fprintf('Regenerated Energy: %.2f kWh\n', Energy_regen);
Eff_avg = mean(eta_trans_arr(eta_trans_arr > 0));
fprintf('Average Transmission Efficiency: %.3f\n', Eff_avg);
fprintf('Estimated Full Range: %.1f km\n', range_est);