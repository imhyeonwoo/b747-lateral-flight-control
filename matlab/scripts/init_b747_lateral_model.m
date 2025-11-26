%% init_b747_lateral_model.m
% B-747 횡방향(lateral) 선형 상태공간 모델 초기화
% - state vector: x = [beta; r; p; phi]
% - input: u = [delta_r; delta_a]
% - output: y = [r; phi]

clear; clc;

%% 상태공간 행렬 정의

% A 행렬
A = [ -0.0558  -0.9968   0.0802   0.0415 ;
       0.598   -0.115   -0.0318   0      ;
      -3.05     0.388   -0.465    0      ;
       0        0.0805   1        0      ];

% B 행렬 (입력: [delta_r; delta_a])
B = [ 0.00729   0       ;
     -0.475     0.00775;
      0.155     0.143  ;
      0         0      ];

% C 행렬 (출력: [r; phi])
C = [ 0  1  0  0;   % 출력 1: yaw rate r
      0  0  0  1];  % 출력 2: bank angle phi

% D 행렬
D = zeros(2, 2);
%% 입력–출력 설정 및 Transfer Function 도출
% 상태공간 시스템 생성
sys = ss(A, B, C, D);

% 이름 달아두기
sys.StateName  = {'\beta', 'r', 'p', '\phi'};
sys.InputName  = {'\delta_r', '\delta_a'};
sys.OutputName = {'r', '\phi'};

% 2x2 전달함수 행렬 G(s) = C (sI - A)^(-1) B
G = tf(sys);

% 채널별 SISO 전달함수 분리
% G11(s) = r(s)/delta_r(s)
% G12(s) = r(s)/delta_a(s)
% G21(s) = phi(s)/delta_r(s)
% G22(s) = phi(s)/delta_a(s)
G11 = G(1,1);   % delta_r -> r
G12 = G(1,2);   % delta_a -> r
G21 = G(2,1);   % delta_r -> phi
G22 = G(2,2);   % delta_a -> phi

% Transfer Function 출력
%disp('=== Transfer function matrix G(s) ===');
%G

%disp('G11(s) = r / delta_r');  G11
%disp('G12(s) = r / delta_a');  G12
%disp('G21(s) = phi / delta_r');G21
%disp('G22(s) = phi / delta_a');G22
