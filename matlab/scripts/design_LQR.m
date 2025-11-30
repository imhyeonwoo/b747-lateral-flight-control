%% Design_LQR.m
% B-747 횡방향(Lateral) LQR 제어기 설계

%% 1. 시스템 행렬 정의 (B-747 Lateral Model)
% 상태 벡터 x = [beta; r; p; phi]
% 입력 벡터 u = [delta_r; delta_a]

A = [-0.0558  -0.9968   0.0802   0.0415 ;
      0.598   -0.115   -0.0318   0      ;
     -3.05     0.388   -0.465    0      ;
      0        0.0805   1        0      ];

B = [ 0.00729   0       ;
     -0.475     0.00775 ;
      0.155     0.143   ;
      0         0       ];

% [중요] LQR 설계를 위한 Full-State 출력 행렬
% LQR은 모든 상태변수(beta, r, p, phi)를 피드백 받아야 하므로
% 출력 행렬 C를 4x4 단위 행렬(Identity Matrix)로 설정합니다.
C_lqr = eye(4); 
D_lqr = zeros(4, 2);

%% 2. 가중치 행렬 (Weighting Matrices) 선정
% J = integral( x'Qx + u'Ru ) dt
% 튜닝 포인트: 이 값들을 조절하여 성능을 바꿉니다.

% --- Q 행렬 (상태 변수 오차에 대한 가중치) ---
% x = [beta(1), r(2), p(3), phi(4)]
q_beta = 1;     % 사이드슬립 (적당히 억제)
q_r    = 100;   % Yaw Rate (더치 롤 억제를 위해 중요 -> 높게 설정)
q_p    = 1;     % Roll Rate (적당히 억제)
q_phi  = 100;   % Bank Angle (자세 유지 및 추종 핵심 -> 높게 설정)

Q = diag([q_beta, q_r, q_p, q_phi]);

% --- R 행렬 (제어 입력 사용량에 대한 가중치) ---
% u = [delta_r(1), delta_a(2)]
% 값이 클수록 입력을 아끼고(천천히 반응), 작을수록 과감하게 씀(빠른 반응)
r_rudder  = 10; 
r_aileron = 10;

R = diag([r_rudder, r_aileron]);

%% 3. LQR 게인 K 계산
% lqr 함수: 최적 이득 행렬 K, 리카티 해 S, 폐루프 극점 E 반환
[K_lqr, S, E] = lqr(A, B, Q, R);

%% ========================================================================
%% [6.3 Final Comparison] Robust Plot (Fix Dimension Error)
%% ========================================================================

% 데이터 인덱스 (사용자 환경에 맞춤)
idx_r_o = 1;    % Open/Washout r 인덱스
idx_r_lqr = 2;  % LQR r 인덱스

% 그래프 스타일
line_in  = 'k-';
line_o   = 'b:';
line_wo  = 'g--';
line_lqr = 'r-';

%% 1. Figure 1: Rudder Step Response Comparison
try
    fig1 = figure('Name', 'Figure 1: Rudder Step Response', 'Color', 'w');
    set(fig1, 'Position', [100, 100, 900, 600]);
    
    % 각 데이터 별로 시간축과 데이터를 따로 가져옵니다. (에러 방지 핵심!)
    % 1) Open-loop
    to  = out_o.rudder_step_out.Time;
    yo  = out_o.rudder_step_out.Data(:, idx_r_o);
    
    % 2) Wash-out
    two = out_wo.rudder_step_out.Time;
    ywo = out_wo.rudder_step_out.Data(:, idx_r_o);
    
    % 3) LQR
    tlqr = out_lqr.rudder_step_out.Time;
    ylqr = out_lqr.rudder_step_out.Data(:, idx_r_lqr);
    
    % 4) Step 입력 (LQR 시간축 기준)
    u_step = ones(size(tlqr)); 
    u_step(1) = 0; 
    
    % 플롯 (각자 자기 시간축 사용)
    plot(tlqr, u_step, line_in, 'LineWidth', 1.2); hold on;
    plot(to,   yo,     line_o,  'LineWidth', 1.5);
    plot(two,  ywo,    line_wo, 'LineWidth', 1.5);
    plot(tlqr, ylqr,   line_lqr,'LineWidth', 2.5);
    
    grid on;
    legend('Step Input (\delta_r)', 'Open-loop (r)', 'Wash-out (r)', 'LQR (r)', 'Location', 'best');
    title('Yaw Rate Response to Rudder Step Input (\delta_r \rightarrow r)');
    xlabel('Time (sec)'); ylabel('Yaw Rate (deg/s)');
    ylim([-8 2]); % 범위 조정

catch ME
    fprintf('Error in Figure 1: %s\n', ME.message);
end

%% 2. Figure 2: Rudder Doublet Response Comparison
try
    fig2 = figure('Name', 'Figure 2: Rudder Doublet Response', 'Color', 'w');
    set(fig2, 'Position', [150, 150, 900, 600]);
    
    % 데이터 추출
    to  = out_o.rudder_doublet_out.Time;
    yo  = out_o.rudder_doublet_out.Data(:, idx_r_o);
    
    two = out_wo.rudder_doublet_out.Time;
    ywo = out_wo.rudder_doublet_out.Data(:, idx_r_o);
    
    tlqr = out_lqr.rudder_doublet_out.Time;
    ylqr = out_lqr.rudder_doublet_out.Data(:, idx_r_lqr);
    
    % Doublet 입력 (시각적 참고용)
    u_doublet = zeros(size(tlqr));
    u_doublet(tlqr>=1 & tlqr<2) = 1;
    u_doublet(tlqr>=2 & tlqr<3) = -1;
    
    % 플롯
    plot(tlqr, u_doublet, line_in, 'LineWidth', 1.2); hold on;
    plot(to,   yo,        line_o,  'LineWidth', 1.5);
    plot(two,  ywo,       line_wo, 'LineWidth', 1.5);
    plot(tlqr, ylqr,      line_lqr,'LineWidth', 2.5);
    
    grid on;
    legend('Doublet Input (\delta_r)', 'Open-loop (r)', 'Wash-out (r)', 'LQR (r)', 'Location', 'best');
    title('Yaw Rate Response to Rudder Doublet Input (\delta_r \rightarrow r)');
    xlabel('Time (sec)'); ylabel('Yaw Rate (deg/s)');
    
    xlim([0 T_end]); ylim([-1.5 1.5]);

catch ME
    fprintf('Error in Figure 2: %s\n', ME.message);
end