%%
%% Mini-Auto-Drive
%%
%% Vehicle Dynamics Parameters
%%
%% Copyright (C) 2017-2022, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

%% Global Variables (required by mbc_car_display)
global P_p_l P_p_frontaxle_1 P_p_rearaxle_1 P_p_rear_1 P_p_c_1 P_p_c_2 P_p_delta_max P_p_psi0 P_p_s10 P_p_s20

%% Geometry
P_p_rearaxle_1 = 49e-3; % distance between IMU and rear axle [ m ]
P_p_frontaxle_1 = 50e-3; % distance between IMU and front axle[ m ]
P_p_rear_1 = -34e-3; % position of rear end of car [ m ]
P_p_c_1 = 167e-3; % length of car [ m ]
P_p_c_2 = 83e-3; % width of car [ m ]

%% Vehicle Parameters
P_p_v_min = 0.15; % speed threshold to switch from dynamics model to
                  % kinematics model to avoid singularities [ m/s ]
P_p_m = 132e-3; % mass [ kg ]
P_p_J = 192e-6; % moment of inertia (yaw) [ kg*m^2 ]

%% Magic Formula Coefficients
P_p_Br = 0.7;
P_p_Cr = 2;
P_p_Dr = 2.5;
P_p_Er = -0.05;
P_p_Bf = 0.7;
P_p_Cf = 2;
P_p_Df = 2;
P_p_Ef = -0.1;

%% Longitudinal Dynamics
P_p_k = 2.51; % gain [ m/s ]
P_p_T = 316e-3; % time constant [ s ]
P_p_uTt = 44e-3; % input dead time [ s ] 
P_p_un_max = 1; % maximum motor input signal [ 1 ] 
P_p_un_min = -P_p_un_max; % minimum motor input signal [ 1 ]
P_p_un_slow_max = 0.328; % maximum motor input signal [ 1 ] 
P_p_un_slow_min = -P_p_un_max; % minimum motor input signal [ 1 ]
P_p_un_cmd_en = true; % enable CmdHalt, CmdForward, CmdReverse

%% Lateral Dynamics
P_p_delta_max = 21.58/180*pi; % maximum steering angle [ rad ]
P_p_delta_min = -P_p_delta_max; % minimum steering angle [ rad ]
P_p_delta_bias = 0; % bias in steering [ 1 ]
P_p_deltan_max = 1; % maximum normalized steering angle [ 1 ]
P_p_deltan_min = -P_p_deltan_max; % minimum normalized steering angle [ 1 ]
P_p_EG = 0; % Eigenlenkgradient [ 1 ]
P_p_l = abs(P_p_frontaxle_1 + P_p_rearaxle_1); % wheel base [ m ]
P_p_lr = P_p_rearaxle_1;
P_p_lf = P_p_frontaxle_1;
P_p_delta_Tt = P_p_uTt; % servo dead time [ s ]

%% Longitudinal disturbance
P_p_un_friction_kd0 = 0.2; % dead zone
P_p_un_friction_kd1 = 0.0; % curve resistance factor

%% Computer vision
P_p_output_Tt = 66e-3; % image processing dead time [ s ]
P_p_sstd = 0; % 1e-3; % [ m ];
P_p_psistd = 0; % deg2rad(1); % [ rad ];
%P_p_vstd = 0.3; % [ m/s ];

%% Total dead time
P_p_Tt = P_p_uTt + P_p_output_Tt;

%% Initial Conditions
% carid 0 
P_p_v0 = 0;     % speed [ m/s ]
P_p_s10 = 1.3;  % s1 position [ m ]
P_p_s20 = 0.2;  % s2 position [ m ]
P_p_psi0 = 0;   % yaw angle [ rad ]
% carid 1
P_p_s11 = 1.3;  % s1 position [ m ]
P_p_s21 = 1.6;  % s2 position [ m ]
P_p_psi1 = pi;  % yaw angle [ rad ]
% carid 2
P_p_s12 = 1.3;  % s1 position [ m ]
P_p_s22 = 0.35; % s2 position [ m ]
P_p_psi2 = 0.0; % yaw angle [ rad ]

% vc1, s1, s2, psi, dpsi/dt, vc2, path length x
% Magic Tire Model (Section 5.5)
P_p_xvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0; 0; 0 ];
P_p_xvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0; 0; 0 ];
P_p_xvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0; 0; 0 ];
% v, s1, s2, psi, dpsi/dt, beta, path length x
P_p_xkinvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0 ];
P_p_xkinvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0 ];
P_p_xkinvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0 ];