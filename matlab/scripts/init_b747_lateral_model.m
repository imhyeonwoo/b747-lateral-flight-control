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