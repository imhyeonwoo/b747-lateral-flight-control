%% Wash-out Filter Parameter Design

%% 1. 파라미터 설정 (Tuning Parameters)
% Wash-out 필터의 시정수 (Time Constant)
% 의미: 저주파(선회) 성분을 차단할 기준 시간. 보통 2~5초 사이로 설정.
% tau가 클수록 더 낮은 주파수까지 통과시키고(차단 영역이 줄어듦),
% tau가 작을수록 더 많은 성분을 차단합니다.
tau_wo = 3.0; % [sec] (초기 추천값: 3.0)

% 피드백 이득 (Feedback Gain)
% 의미: Yaw Damper가 얼마나 강하게 개입할지 결정.
% 값이 너무 크면 진동이 생기거나 Rudder가 과하게 움직일 수 있습니다.
K_wo = -1.5;   % (초기 추천값: 0.5 ~ 2.0 사이에서 조절 필요)

%% 2. 전달함수 계수 생성 (For Simulink)
% Wash-out Filter Transfer Function:
% H(s) = (tau * s) / (tau * s + 1)
% Numerator(분자): [tau, 0]   -> tau * s
% Denominator(분모): [tau, 1] -> tau * s + 1

num_wo = [tau_wo, 0];
den_wo = [tau_wo, 1];

%% 3. 설계 결과 확인 (Bode Plot) - 선택사항
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

%% Visualization : Open-Loop vs Wash-out Filter Closed-Loop Comparison
% 각 입력(Step, Impulse, 3-2-1-1, Doublet)에 대해
% Open-loop와 Closed-loop의 Yaw Rate(r)를 하나의 그래프로 겹쳐서 비교합니다.

%% 1. Step Response Comparison (Yaw Rate Only)
figure('Name', 'Step Response Comparison', 'Color', 'w');
plot(out_o.rudder_step_out.Time, out_o.rudder_step_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_step_out.Time, out_wo.rudder_step_out.Data(:,2), 'r-', 'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Step Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]); % 진동이 잡히는 초기 구간을 자세히 보기 위해 시간 축 조정 (필요시 수정)


%% 2. Impulse Response Comparison (Yaw Rate Only)
figure('Name', 'Impulse Response Comparison', 'Color', 'w');
plot(out_o.rudder_impulse_out.Time, out_o.rudder_impulse_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_impulse_out.Time, out_wo.rudder_impulse_out.Data(:,2), 'r-', 'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Impulse Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);


%% 3. 3-2-1-1 Response Comparison (Yaw Rate Only)
figure('Name', '3-2-1-1 Response Comparison', 'Color', 'w');
plot(out_o.rudder_3211_out.Time, out_o.rudder_3211_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_3211_out.Time, out_wo.rudder_3211_out.Data(:,2), 'r-', 'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - 3-2-1-1 Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);


%% 4. Doublet Response Comparison (Yaw Rate Only)
figure('Name', 'Doublet Response Comparison', 'Color', 'w');
plot(out_o.rudder_doublet_out.Time, out_o.rudder_doublet_out.Data(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(out_wo.rudder_doublet_out.Time, out_wo.rudder_doublet_out.Data(:,2), 'r-', 'LineWidth', 1.5);
grid on;
legend('Open-Loop (No Filter)', 'Closed-Loop (With Wash-out)', 'Location', 'best');
title('Yaw Rate (r) Comparison - Doublet Input');
ylabel('Yaw Rate (deg/s)');
xlabel('Time (sec)');
xlim([0 T_end]);