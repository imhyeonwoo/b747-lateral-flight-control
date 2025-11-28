%% init_b747_compensator.m
% Root locus로 결정된 보상기 파라미터 (최종본)

s = tf('s');

% 루트 궤적으로 잡은 최종 파라미터 (나중에 값만 바꿔 쓰면 됨)
K_r = 0.8;   z_r = 0.05;  p_r = 0.7;   % rudder → yaw loop
K_a = 0.6;   z_a = 0.02;  p_a = 0.4;   % aileron → bank loop

% 보상기 "형태" (gain 제외) : C_r(s) = (s+z_r)/(s+p_r), C_a(s) = (s+z_a)/(s+p_a)
C_r = (s + z_r)/(s + p_r);
C_a = (s + z_a)/(s + p_a);

% 이론 해석용으로는 K_r, K_a를 곱해서 사용
% 예) L_r = K_r * C_r * G11;  L_a = K_a * C_a * G22;
% Simulink에서는 Transfer Fcn 블록에 C_r, C_a의 계수만 넣고
% Gain 블록에 각각 K_r, K_a를 넣는다.

% Simulink Transfer Fcn 블록에 들어갈 계수 (보상기 형태만)
% num = [1  z],  den = [1  p]
num_Cr = [1, z_r];
den_Cr = [1, p_r];

num_Ca = [1, z_a];
den_Ca = [1, p_a];

%% 4.3.1 Rudder step 응답 비교
%% (1) Root locus 비교 (강의자료 스타일 : K 변화 시 폐루프 극 이동)

figure('Name','Rudder loop root locus','NumberTitle','off');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% (a) 보상기 없는 루프 : L(s) = K * G11(s)
nexttile;
rlocus(G11);
grid on;
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
title('Uncompensated: K G_{11}(s)');

% (b) 보상기 포함 루프 : L(s) = K * C_r(s) G11(s)
nexttile;
rlocus(C_r*G11);
grid on;
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
title('Compensated: K C_r(s) G_{11}(s)');
hold on;
% 내가 최종으로 선택한 K_r 위치 표시
s_cl = rlocus(C_r*G11, K_r);      % K = K_r일 때의 폐루프 극
plot(real(s_cl), imag(s_cl), 'r*', 'MarkerSize', 8);
legend('Root locus','Chosen pole (K_r)','Location','best');

%% (2) 전달함수 기반 step 응답 비교 (open-loop 형식: G11 vs C_r(s)G11(s))

T_r_open = G11;          % 원래 플랜트
T_r_comp = C_r * G11;    % 보상기 직렬 연결

T_end  = 800;                    % 시뮬레이션 시간 (3장에서와 동일하게)
t_grid = linspace(0, T_end, 2000);

[y_open, t_out] = step(T_r_open, t_grid);
[y_comp, ~    ] = step(T_r_comp, t_grid);

% rad/s → deg/s 변환
r_open_deg = rad2deg(y_open);
r_comp_deg = rad2deg(y_comp);

figure('Name','Rudder step (theoretical TF)','NumberTitle','off');
plot(t_out, r_open_deg, 'LineWidth', 1.5); hold on;
plot(t_out, r_comp_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate r [deg/s]');
legend('Uncompensated G_{11}(s)', 'With compensator C_r(s)G_{11}(s)', ...
       'Location','best');
title('Rudder step: \delta_r step \rightarrow r (theoretical, open loop)');

%% (3) Simulink 결과 비교 (open-loop 모델 vs 보상기 포함 모델)

ts_open = out_o.rudder_step_out;            % [r, phi] (deg 단위)
ts_comp = out_c.rudder_step_comp_out;

t_sim_open = ts_open.Time;
t_sim_comp = ts_comp.Time;

r_sim_open = ts_open.Data(:,1);   % 1열: yaw rate r [deg/s]
r_sim_comp = ts_comp.Data(:,1);

figure('Name','Rudder step (Simulink)','NumberTitle','off');
plot(t_sim_open, r_sim_open, 'LineWidth',1.5); hold on;
plot(t_sim_comp, r_sim_comp, 'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate r [deg/s]');
legend('Open-loop (Simulink)', 'With compensator (Simulink)', ...
       'Location','best');
title('Rudder step: \delta_r step \rightarrow r (Simulink 결과)');

%% 4.3.2 Aileron step 응답 비교 -> 여기서부터 그냥 전달함수만으로 구하는거 안할 예정_251128
%% (1) Root locus 비교 : aileron → bank loop (G22)

figure('Name','Aileron loop root locus','NumberTitle','off');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% (a) 보상기 없는 루프 : L(s) = K * G22(s)
nexttile;
rlocus(G22);
grid on;
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
title('Uncompensated: K G_{22}(s)');

% (b) 보상기 포함 루프 : L(s) = K * C_a(s) G22(s)
nexttile;
rlocus(C_a*G22);
grid on;
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
title('Compensated: K C_a(s) G_{22}(s)');
hold on;
% 최종으로 선택한 K_a 위치 표시
s_cl_a = rlocus(C_a*G22, K_a);      % K = K_a일 때의 폐루프 극
plot(real(s_cl_a), imag(s_cl_a), 'r*', 'MarkerSize', 8);
legend('Root locus','Chosen pole (K_a)','Location','best');

%% (2) Simulink 결과 비교 (open-loop 모델 vs 보상기 포함 모델)

% To Workspace 변수 이름에 맞게 수정할 것
ts_phi_open = out_o.aileron_step_out;          % [r, phi] (deg 단위)
ts_phi_comp = out_c.aileron_step_comp_out;

t_sim_phi_open = ts_phi_open.Time;
t_sim_phi_comp = ts_phi_comp.Time;

phi_sim_open = ts_phi_open.Data(:,2);   % 2열: bank angle phi [deg]
phi_sim_comp = ts_phi_comp.Data(:,2);

figure('Name','Aileron step (Simulink)','NumberTitle','off');
plot(t_sim_phi_open, phi_sim_open, 'LineWidth',1.5); hold on;
plot(t_sim_phi_comp, phi_sim_comp, 'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Bank angle \phi [deg]');
legend('Open-loop (Simulink)', 'With compensator (Simulink)', ...
       'Location','best');
title('Aileron step: \delta_a step \rightarrow \phi (Simulink 결과)');

%% 4.3.3 Rudder 3-2-1-1 응답 비교 -> 여기서부턴 root-locus 그릴 필요 없음

ts_rud_3211_open = out_o.rudder_3211_out;          % [r, phi] (deg 단위)
ts_rud_3211_comp = out_c.rudder_3211_comp_out;

% 시간 벡터
t_rud_3211_open = ts_rud_3211_open.Time;
t_rud_3211_comp = ts_rud_3211_comp.Time;

% 데이터 (1열: yaw rate r, 2열: bank angle phi)
r_rud_3211_open  = ts_rud_3211_open.Data(:,1);     % [deg/s]
r_rud_3211_comp  = ts_rud_3211_comp.Data(:,1);

% 필요하면 phi도 같이 보고 싶을 때 사용
% phi_rud_3211_open = ts_rud_3211_open.Data(:,2);  % [deg]
% phi_rud_3211_comp = ts_rud_3211_comp.Data(:,2);

figure('Name','Rudder 3-2-1-1 (Simulink)','NumberTitle','off');
plot(t_rud_3211_open, r_rud_3211_open, 'LineWidth',1.5); hold on;
plot(t_rud_3211_comp, r_rud_3211_comp, 'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate r [deg/s]');
legend('Open-loop (Simulink)', 'With compensator (Simulink)', ...
       'Location','best');
title('Rudder 3-2-1-1 input: \delta_r 3-2-1-1 \rightarrow r (Simulink 결과)');

%% 4.3.4 Coupling responses under 3-2-1-1 input
%  - δ_r 3-2-1-1 → φ (rudder → bank angle)
%  - δ_a 3-2-1-1 → r (aileron → yaw rate)

%% (1) δ_r 3-2-1-1 입력에 대한 bank angle φ 응답 비교 (G21 채널)

% Simulink 결과 불러오기 (deg 단위, [r, phi])
ts_rud_3211_open = out_o.rudder_3211_out;         % uncompensated
ts_rud_3211_comp = out_c.rudder_3211_comp_out;    % compensated

% 시간 벡터
t_rud_3211_open = ts_rud_3211_open.Time;
t_rud_3211_comp = ts_rud_3211_comp.Time;

% 데이터 (1열: yaw rate r [deg/s], 2열: bank angle phi [deg])
phi_rud_3211_open = ts_rud_3211_open.Data(:,2);   % [deg]
phi_rud_3211_comp = ts_rud_3211_comp.Data(:,2);   % [deg]

figure('Name','Rudder 3-2-1-1 → phi (Simulink)','NumberTitle','off');
plot(t_rud_3211_open, phi_rud_3211_open, 'LineWidth',1.5); hold on;
plot(t_rud_3211_comp, phi_rud_3211_comp, 'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Bank angle \phi [deg]');
legend('Uncompensated (Simulink)', 'With compensator (Simulink)', ...
       'Location','best');
title('Rudder 3-2-1-1 input: \delta_r 3-2-1-1 \rightarrow \phi (Simulink 결과)');


%% (2) δ_a 3-2-1-1 입력에 대한 yaw rate r 응답 비교 (G12 채널)

% Simulink 결과 불러오기 (deg 단위, [r, phi])
ts_ail_3211_open = out_o.aileron_3211_out;         % uncompensated
ts_ail_3211_comp = out_c.aileron_3211_comp_out;    % compensated

% 시간 벡터
t_ail_3211_open = ts_ail_3211_open.Time;
t_ail_3211_comp = ts_ail_3211_comp.Time;

% 데이터 (1열: yaw rate r [deg/s], 2열: bank angle phi [deg])
r_ail_3211_open = ts_ail_3211_open.Data(:,1);      % [deg/s]
r_ail_3211_comp = ts_ail_3211_comp.Data(:,1);      % [deg/s]

figure('Name','Aileron 3-2-1-1 → r (Simulink)','NumberTitle','off');
plot(t_ail_3211_open, r_ail_3211_open, 'LineWidth',1.5); hold on;
plot(t_ail_3211_comp, r_ail_3211_comp, 'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate r [deg/s]');
legend('Uncompensated (Simulink)', 'With compensator (Simulink)', ...
       'Location','best');
title('Aileron 3-2-1-1 input: \delta_a 3-2-1-1 \rightarrow r (Simulink 결과)');