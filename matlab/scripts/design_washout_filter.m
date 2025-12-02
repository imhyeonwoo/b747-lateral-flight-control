%% Wash-out Filter Parameter Design
%  (for Section 5: Yaw-damper with wash-out filter)

s = tf('s');

%% 0. Yaw loop plant 선택
% Rudder -> Yaw rate 경로에 대해 yaw-damper를 건다고 가정.
% 4장에서 설계한 rudder 보상기 C_r(s)를 포함한 루프를 대상으로 하면:
G_yaw = C_r * G11;     % (원하면 G11 단독으로 바꿔서 실험 가능)

%% 1. 파라미터 설정 (Tuning Parameters)
% Wash-out 필터의 시정수 (Time Constant)
% 의미: 저주파(선회) 성분을 차단할 기준 시간. 보통 2~5초 사이로 설정.
tau_wo = 3.0;          % [sec] (초기 값: 3.0)

% 피드백 이득 (Feedback Gain)
% 의미: Yaw Damper가 얼마나 강하게 개입할지 결정.
% 구조가 δ_r,tot = δ_r,cmd - K_wo * H_wo(s) * r 이므로
% 안정화를 위해 K_wo < 0 로 두고, root locus에서는 |K_wo|를 사용.
K_wo = -1.5;           % 실제 적용 이득 (부호 포함)
K_wo_mag = abs(K_wo);  % root locus에서 사용할 양의 이득 크기

%% 2. Wash-out 필터 "형태" 정의 (이득 제외)
% H_wo,shape(s) = (tau * s) / (tau * s + 1)
H_wo_shape = (tau_wo * s) / (tau_wo * s + 1);

%% 3. Yaw-damper 루프 Root Locus (K_wo 설계용)
% 특성방정식: 1 - K_wo * H_wo_shape * G_yaw = 0
% -> root locus 기준: 1 + K * L(s) = 0 with L(s) = -H_wo_shape * G_yaw
L_yaw_wo = -H_wo_shape * G_yaw;

figure('Name','Yaw-damper root locus with wash-out filter','NumberTitle','off');
h_rl = rlocusplot(L_yaw_wo);
setoptions(h_rl,'Grid','on');
xlabel('Real axis (s^{-1})');
ylabel('Imag axis (s^{-1})');
title('Root locus of  K_{wo} H_{wo}(s) C_r(s) G_{11}(s)');

hold on;
% 현재 선택된 |K_wo|에서의 폐루프 극점 표시
poles_wo = rlocus(L_yaw_wo, K_wo_mag);
plot(real(poles_wo), imag(poles_wo), 'r*', 'MarkerSize', 8, 'LineWidth', 1.2);
legend('Root locus','Closed-loop poles at |K_{wo}|','Location','best');

% 필요하면 여기에서 원하는 극점 위치 s_des를 정한 뒤,
%   [K_wo_mag_new, poles_new] = rlocfind(L_yaw_wo, s_des);
% 로 새로운 이득 크기를 계산하고,
%   K_wo = -K_wo_mag_new;
% 로 업데이트하면 된다.

%% 4. 전달함수 계수 생성 (For Simulink)
% 실제 Wash-out Filter Transfer Function (이득 제외)
% H_wo(s) = (tau * s) / (tau * s + 1)
num_wo = [tau_wo, 0];
den_wo = [tau_wo, 1];

% Simulink에서는 Transfer Fcn 블록에 num_wo/den_wo를 넣고,
% 그 앞에 Gain 블록으로 K_wo (음수!) 를 둔다.

%% 5. 설계 결과 확인 (Bode Plot) - 선택사항
% 필터가 제대로 고역통과(High-pass) 특성을 가지는지 확인
figure;
w = logspace(-2, 2, 1000);
H_wo = tf(num_wo, den_wo);
bode(H_wo, w);
grid on;
title('Bode Plot of Designed Wash-out Filter');
fprintf('Wash-out Filter Design Complete.\n');
fprintf('Time Constant (tau): %.2f sec\n', tau_wo);
fprintf('Cut-off Frequency: %.2f rad/s\n', 1/tau_wo);

%% 6. Visualization : Open-Loop vs Wash-out Filter Closed-Loop Comparison
% 각 입력(Step, Impulse, 3-2-1-1, Doublet)에 대해
% Open-loop와 Closed-loop의 Yaw Rate(r)를 하나의 그래프로 겹쳐서 비교합니다.

%% 6-1. Step Response Comparison (Yaw Rate Only)
figure('Name', 'Step Response Comparison', 'Color', 'w');
plot(out_o.rudder_step_out.Time,  out_o.rudder_step_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_step_out.Time, out_wo.rudder_step_out.Data(:,2), 'r-',  'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Step Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);

%% 6-2. Impulse Response Comparison (Yaw Rate Only)
figure('Name', 'Impulse Response Comparison', 'Color', 'w');
plot(out_o.rudder_impulse_out.Time,  out_o.rudder_impulse_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_impulse_out.Time, out_wo.rudder_impulse_out.Data(:,2), 'r-',  'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Impulse Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);

%% 6-3. 3-2-1-1 Response Comparison (Yaw Rate Only)
figure('Name', '3-2-1-1 Response Comparison', 'Color', 'w');
plot(out_o.rudder_3211_out.Time,  out_o.rudder_3211_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_3211_out.Time, out_wo.rudder_3211_out.Data(:,2), 'r-',  'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - 3-2-1-1 Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);

%% 6-4. Doublet Response Comparison (Yaw Rate Only)
figure('Name', 'Doublet Response Comparison', 'Color', 'w');
plot(out_o.rudder_doublet_out.Time,  out_o.rudder_doublet_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_doublet_out.Time, out_wo.rudder_doublet_out.Data(:,2), 'r-',  'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Doublet Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);
