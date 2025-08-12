%%
%% Mini-Auto-Drive MAD76
%%
%% Vehicle Dynamics Parameters
%%
%% Copyright (C) 2017-2024, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

%% Global Variables (required by mbc_car_display)
global P_p_l P_p_frontaxle_1 P_p_rearaxle_1 P_p_rear_1 P_p_c_1 P_p_c_2 P_p_delta_max P_p_psi0 P_p_s10 P_p_s20

%% Vehicle Parameters
P_p_v_min = 1.0; % speed threshold to switch from dynamics model to
                  % kinematics model to avoid singularities [ m/s ]
                  % high value --> never use dynamics model, always
                  % kinematics only
P_p_m = 13e-3; % mass [ kg ]
P_p_J = 3e-6; % moment of inertia (yaw) [ kg*m^2 ]

%% Magic Formula Coefficients
P_p_Br = 0.6;
P_p_Cr = 1.0;
P_p_Dr = 1.0;
P_p_Er = 0.0;
P_p_Bf = 0.4;
P_p_Cf = 1.0;
P_p_Df = 1.0;
P_p_Ef = 0.0;

%% Longitudinal Dynamics
% car 00
P_p_k = 3.4; % gain [ m/s ]
P_p_T = 120e-3; % time constant [ s ]
P_p_uTt = 100e-3; % input dead time [ s ] 
P_p_un_max = 0.2; % maximum motor input signal [ 1 ] 
P_p_un_min = -P_p_un_max; % minimum motor input signal [ 1 ]

% car 10
% P_p_k = 3.2; % gain [ m/s ]
% P_p_T = 90e-3; % time constant [ s ]
% P_p_uTt = 125e-3; % input dead time [ s ] 
% P_p_un_max = 0.2; % maximum motor input signal [ 1 ] 
% P_p_un_min = -P_p_un_max; % minimum motor input signal [ 1 ]

P_p_un_cmd_en = false; % enable CmdHalt, CmdForward, CmdReverse

%% Longitudinal disturbance
% v = [ 0.3 0.3 0.2 0.2 0.4 0.5 0.5 ];
% deltan = [ 0.93 0.0 0.93 0.0 0.93 0.0 ];
% un = [ 0.12 0.105 0.09 0.08 0.17 0.14 ];
% increase in un when full steering: approx factor 1.15
P_p_un_friction_kd0 = 0.04; % dead zone
P_p_un_friction_kd1 = 0.15; % curve resistance factor

%% Lateral Dynamics
P_p_delta_max = 22/180*pi; % maximum steering angle [ rad ]
P_p_delta_min = -P_p_delta_max; % minimum steering angle [ rad ]
P_p_delta_bias = 0; % bias in steering [ 1 ]
P_p_deltan_max = 0.93; % maximum normalized steering angle [ 1 ]
P_p_deltan_min = -P_p_deltan_max; % minimum normalized steering angle [ 1 ]
P_p_EG = 0.02; % Eigenlenkgradient [ 1 ]
P_p_l = 32.5e-3; % wheel base [ m ]
P_p_lr = 0.5 * P_p_l;
P_p_lf = 0.5 * P_p_l;
P_p_delta_Tt = P_p_uTt; % servo dead time [ s ]

%% Geometry
P_p_rearaxle_1 = P_p_lr; % distance between IMU and rear axle [ m ]
P_p_frontaxle_1 = P_p_lf; % distance between IMU and front axle[ m ]
P_p_c_1 = 49e-3; % length of car [ m ]
P_p_c_2 = 22e-3; % width of car [ m ]
P_p_rear_1 = -6e-3; % position of rear end of car (from rear axle) [ m ]

%% Computer vision
P_p_output_Tt = 0e-3; % image processing dead time [ s ]
P_p_sstd = 0; %1e-3; % [ m ];
P_p_psistd = 0; %deg2rad(1); % [ rad ];
%P_p_vstd = 0.3; % [ m/s ];

%% Total dead time
P_p_Tt = P_p_uTt + P_p_output_Tt;

%% Initial Conditions
% carid 0 
P_p_v0 = 0;     % speed [ m/s ]
P_p_s10 = 0.5;  % s1 position [ m ]
P_p_s20 = 0.38;  % s2 position [ m ]
P_p_psi0 = 0;   % yaw angle [ rad ]
% carid 1
P_p_s11 = 0.3;  % s1 position [ m ]
P_p_s21 = 0.45;  % s2 position [ m ]
P_p_psi1 = 0;  % yaw angle [ rad ]
% carid 2
P_p_s12 = 0.3;  % s1 position [ m ]
P_p_s22 = 0.35; % s2 position [ m ]
P_p_psi2 = 0.0; % yaw angle [ rad ]
% carid 3
P_p_s13 = 0.3;  % s1 position [ m ]
P_p_s23 = 0.3; % s2 position [ m ]
P_p_psi3 = 0.0; % yaw angle [ rad ]

% vc1, s1, s2, psi, dpsi/dt, vc2, path length x
% Magic Tire Model (Section 5.5)
P_p_xvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0; 0; 0 ];
P_p_xvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0; 0; 0 ];
P_p_xvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0; 0; 0 ];
P_p_xvec3 = [ P_p_v0; P_p_s13; P_p_s23; P_p_psi3; 0; 0; 0 ];
% v, s1, s2, psi, dpsi/dt, beta, path length x
P_p_xkinvec0 = [ P_p_v0; P_p_s10; P_p_s20; P_p_psi0; 0 ];
P_p_xkinvec1 = [ P_p_v0; P_p_s11; P_p_s21; P_p_psi1; 0 ];
P_p_xkinvec2 = [ P_p_v0; P_p_s12; P_p_s22; P_p_psi2; 0 ];
P_p_xkinvec3 = [ P_p_v0; P_p_s13; P_p_s23; P_p_psi3; 0 ];