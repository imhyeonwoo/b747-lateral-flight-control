%% open_loop_step_responses.m
% Simulink 결과(out 구조체)를 이용한 B-747 lateral step responses 플롯

%% 1. Rudder step: [delta_r step, delta_a = 0] -> [r, phi]

ts_rudder = out.rudder_step_out;     % timeseries 객체
t_rud  = ts_rudder.Time;             % 시간 벡터
y_rud  = ts_rudder.Data;             % [N x 2] (r, phi) 라고 가정

r_rud   = y_rud(:,1);                % yaw rate r (deg/s)
phi_rud = y_rud(:,2);                % bank angle phi (deg)

%% 2. Aileron step: [delta_r = 0, delta_a step] -> [r, phi]

ts_ail = out.aileron_step_out;
t_ail  = ts_ail.Time;
y_ail  = ts_ail.Data;

r_ail   = y_ail(:,1);
phi_ail = y_ail(:,2);

%% 3. 플롯 (Rudder step 좌, Aileron step 우)

figure('Name','B747 Lateral Step Responses (from Simulink)','NumberTitle','off');
tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

% --- (1) Rudder step ---
nexttile(1);
plot(t_rud, r_rud,   'LineWidth',1.5); hold on;
plot(t_rud, phi_rud, 'LineWidth',1.5);
grid on;
title('Rudder step: [\delta_r step, \delta_a = 0] \rightarrow [r, \phi]');
xlabel('Time [s]');
ylabel('Amplitude [deg, deg/s]');
legend('Yaw rate r','Bank angle \phi','Location','best');

% --- (2) Aileron step ---
nexttile(2);
plot(t_ail, r_ail,   'LineWidth',1.5); hold on;
plot(t_ail, phi_ail, 'LineWidth',1.5);
grid on;
title('Aileron step: [\delta_r = 0, \delta_a step] \rightarrow [r, \phi]');
xlabel('Time [s]');
ylabel('Amplitude [deg, deg/s]');
legend('Yaw rate r','Bank angle \phi','Location','best');

sgtitle('Open-loop Step Responses of B-747 Lateral Model (Simulink 결과)');

%% 4. MATLAB 내장 step() 기반 이론적 응답 (비교용)

% init_b747_lateral_model.m 을 실행해서 G11~G22 가 작업공간에 있다고 가정
if exist('G11','var') && exist('G12','var') && exist('G21','var') && exist('G22','var')

    % --- Rudder: delta_r step -> r, phi ---
    tFinal_rud = t_rud(end);   % Simulink 사용 시간 범위와 맞추기
    [y_r_dr_step_tf,   t_rud_tf] = step(G11, tFinal_rud);
    [y_phi_dr_step_tf, ~       ] = step(G21, tFinal_rud);

    % --- Aileron: delta_a step -> r, phi ---
    tFinal_ail = t_ail(end);
    [y_r_da_step_tf,   t_ail_tf] = step(G12, tFinal_ail);
    [y_phi_da_step_tf, ~       ] = step(G22, tFinal_ail);

    % rad -> deg (Simulink 결과에 맞추기)
    y_r_dr_step_tf_deg   = rad2deg(y_r_dr_step_tf);
    y_phi_dr_step_tf_deg = rad2deg(y_phi_dr_step_tf);
    y_r_da_step_tf_deg   = rad2deg(y_r_da_step_tf);
    y_phi_da_step_tf_deg = rad2deg(y_phi_da_step_tf);

    % === 두 번째 figure: MATLAB step() 결과만 ===
    figure('Name','B747 Lateral Step Responses (MATLAB step)','NumberTitle','off');
    tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

    % --- (1) Rudder step (tf/step) ---
    nexttile(1);
    plot(t_rud_tf, y_r_dr_step_tf_deg,   'LineWidth',1.5); hold on;
    plot(t_rud_tf, y_phi_dr_step_tf_deg, 'LineWidth',1.5);
    grid on;
    title('Rudder step (tf/step): [\delta_r step, \delta_a = 0] \rightarrow [r, \phi]');
    xlabel('Time [s]');
    ylabel('Amplitude [deg, deg/s]');
    legend('Yaw rate r','Bank angle \phi','Location','best');

    % --- (2) Aileron step (tf/step) ---
    nexttile(2);
    plot(t_ail_tf, y_r_da_step_tf_deg,   'LineWidth',1.5); hold on;
    plot(t_ail_tf, y_phi_da_step_tf_deg, 'LineWidth',1.5);
    grid on;
    title('Aileron step (tf/step): [\delta_r = 0, \delta_a step] \rightarrow [r, \phi]');
    xlabel('Time [s]');
    ylabel('Amplitude [deg, deg/s]');
    legend('Yaw rate r','Bank angle \phi','Location','best');

    sgtitle('Open-loop Step Responses of B-747 Lateral Model (MATLAB step 함수 결과)');

else
    warning('G11~G22 transfer functions not found. init_b747_lateral_model.m 을 먼저 실행하세요.');
end