%% init_b747_compensator.m
% Root locus로 결정된 보상기 파라미터 (최종본)

s = tf('s');

% 루트 궤적으로 잡은 최종 파라미터 (나중에 값만 바꿔 쓰면 됨)
K_r = 0.2729;   z_r = 0.05;  p_r = 0.7;   % rudder → yaw loop
K_a = 1.25;   z_a = 0.02;  p_a = 0.4;   % aileron → bank loop

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

%% 4.3.1 Rudder step 응답 비교  (G11 : δ_r → r 루프)

%% (1) Root locus 비교 : K 변화 시 폐루프 극 이동

% 루프 전달함수 (gain K는 rlocus 안에서 변수로 취급)
L_u = G11;        % uncompensated : K * G11(s)
L_c = C_r * G11;  % compensated   : K * C_r(s) G11(s)

figure('Name','Rudder loop root locus','NumberTitle','off');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% (a) 보상기 없는 루프
nexttile;
h_u = rlocusplot(L_u);
setoptions(h_u,'Grid','on');
title('Uncompensated: K G_{11}(s)');
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');

% (b) 보상기 포함 루프
nexttile;
h_c = rlocusplot(L_c);
setoptions(h_c,'Grid','on');
title('Compensated: K C_r(s) G_{11}(s)');
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
hold on;

% K = K_r 에서의 폐루프 극점 위치 표시 (붉은 별표)
s_cl = rlocus(L_c, K_r);             % K = K_r일 때 폐루프 극
plot(real(s_cl), imag(s_cl), 'r*', ...
     'MarkerSize', 8, 'LineWidth', 1.2);

legend('Root locus','Closed-loop poles at K_r','Location','best');

%% (2) Simulink 결과 비교 (폐루프 모델 vs 보상기 포함 폐루프 모델)

% out_o : G11 기반 폐루프 (보상기 없음)
% out_c : Cr(s) 포함 폐루프 (보상기 적용)
ts_open = out_o.rudder_step_out;         % [r, phi] (deg 단위)
ts_comp = out_c.rudder_step_comp_out;

t_sim_open = ts_open.Time;
t_sim_comp = ts_comp.Time;

r_sim_open = ts_open.Data(:,1);          % 1열: r [deg/s]
r_sim_comp = ts_comp.Data(:,1);

figure('Name','Rudder step (Simulink closed loop)','NumberTitle','off');
plot(t_sim_open, r_sim_open,'LineWidth',1.5); hold on;
plot(t_sim_comp, r_sim_comp,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate r [deg/s]');
legend('Uncompensated closed loop (Simulink)', ...
       'Compensated closed loop (Simulink)', ...
       'Location','best');
title('Rudder step: r_{cmd} step \rightarrow r (Simulink)');

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
