%% open_loop_impulse_responses.m
% Simulink 결과(out 구조체)를 이용한 B-747 lateral impulse responses 플롯

%% 1. Rudder impulse: [delta_r impulse, delta_a = 0] -> [r, phi]

ts_rudder_imp = out.rudder_impulse_out;   % timeseries 객체
t_rud  = ts_rudder_imp.Time;              % 시간 벡터
y_rud  = ts_rudder_imp.Data;              % [N x 2] (r, phi) 라고 가정

r_rud   = y_rud(:,1);                     % yaw rate r (deg/s)
phi_rud = y_rud(:,2);                     % bank angle phi (deg)

%% 2. Aileron impulse: [delta_r = 0, delta_a impulse] -> [r, phi]

ts_ail_imp = out.aileron_impulse_out;
t_ail  = ts_ail_imp.Time;
y_ail  = ts_ail_imp.Data;

r_ail   = y_ail(:,1);
phi_ail = y_ail(:,2);

%% 3. 플롯 (Rudder impulse 좌, Aileron impulse 우)

figure('Name','B747 Lateral Impulse Responses (from Simulink)','NumberTitle','off');
tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

% --- (1) Rudder impulse ---
nexttile(1);
plot(t_rud, r_rud,   'LineWidth',1.5); hold on;
plot(t_rud, phi_rud, 'LineWidth',1.5);
grid on;
title('Rudder impulse: [\delta_r impulse, \delta_a = 0] \rightarrow [r, \phi]');
xlabel('Time [s]');
ylabel('Amplitude [deg, deg/s]');
legend('Yaw rate r','Bank angle \phi','Location','best');

% --- (2) Aileron impulse ---
nexttile(2);
plot(t_ail, r_ail,   'LineWidth',1.5); hold on;
plot(t_ail, phi_ail, 'LineWidth',1.5);
grid on;
title('Aileron impulse: [\delta_r = 0, \delta_a impulse] \rightarrow [r, \phi]');
xlabel('Time [s]');
ylabel('Amplitude [deg, deg/s]');
legend('Yaw rate r','Bank angle \phi','Location','best');

sgtitle('Open-loop Impulse Responses of B-747 Lateral Model (Simulink 결과)');

%% 4. MATLAB 내장 impulse() 기반 이론적 응답 (비교용)

if exist('G11','var') && exist('G12','var') && exist('G21','var') && exist('G22','var')

    % Simulink와 대략 맞추기 위해 종료시간을 동일하게 사용
    tFinal_rud = T_end;
    tFinal_ail = T_end;

    % --- Rudder: delta_r impulse -> r, phi ---
    [y_r_dr_imp_tf,   t_r_dr_tf]   = impulse(G11, tFinal_rud);
    [y_phi_dr_imp_tf, t_phi_dr_tf] = impulse(G21, tFinal_rud);

    % --- Aileron: delta_a impulse -> r, phi ---
    [y_r_da_imp_tf,   t_r_da_tf]   = impulse(G12, tFinal_ail);
    [y_phi_da_imp_tf, t_phi_da_tf] = impulse(G22, tFinal_ail);

    % rad -> deg
    y_r_dr_imp_tf_deg   = rad2deg(y_r_dr_imp_tf);
    y_phi_dr_imp_tf_deg = rad2deg(y_phi_dr_imp_tf);
    y_r_da_imp_tf_deg   = rad2deg(y_r_da_imp_tf);
    y_phi_da_imp_tf_deg = rad2deg(y_phi_da_imp_tf);

    % === 두 번째 figure: MATLAB impulse() 결과만 ===
    figure('Name','B747 Lateral Impulse Responses (MATLAB impulse)', ...
           'NumberTitle','off');
    tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

    % --- (1) Rudder impulse (tf/impulse) ---
    nexttile(1);
    plot(t_r_dr_tf,   y_r_dr_imp_tf_deg,   'LineWidth',1.5); hold on;
    plot(t_phi_dr_tf, y_phi_dr_imp_tf_deg, 'LineWidth',1.5);
    grid on;
    title('Rudder impulse (tf/impulse): [\delta_r imp, \delta_a = 0] \rightarrow [r, \phi]');
    xlabel('Time [s]');
    ylabel('Amplitude [deg, deg/s]');
    legend('Yaw rate r','Bank angle \phi','Location','best');

    % --- (2) Aileron impulse (tf/impulse) ---
    nexttile(2);
    plot(t_r_da_tf,   y_r_da_imp_tf_deg,   'LineWidth',1.5); hold on;
    plot(t_phi_da_tf, y_phi_da_imp_tf_deg, 'LineWidth',1.5);
    grid on;
    title('Aileron impulse (tf/impulse): [\delta_r = 0, \delta_a imp] \rightarrow [r, \phi]');
    xlabel('Time [s]');
    ylabel('Amplitude [deg, deg/s]');
    legend('Yaw rate r','Bank angle \phi','Location','best');

    sgtitle('Open-loop Impulse Responses of B-747 Lateral Model (MATLAB impulse 함수 결과)');

else
    warning('G11~G22 transfer functions not found. init_b747_lateral_model.m 을 먼저 실행하세요.');
end
