%%
%% Mini-Auto-Drive MAD76
%%
%% Motion Controller c71
%%
%% Copyright (C) 2017-2025, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

%% Speed Controller

P_cv_use_smith = uint8(1);                  % use smith predictor instead of PI-controller

% assumed plant dynamics
k = P_p_k;
T = P_p_T;

% Time-discrete plant parameters
P_p_ddel= floor(P_p_Tt/P_dt);               % plant total delay
P_p_b1_Gsv = P_p_k*(1-exp(-P_dt/P_p_T));    % time discrete linear plant without delay
P_p_a1_Gsv = exp(-P_dt/P_p_T);              % Gsv = P_p_b1_Gsv*z^-1 / (1 - p_p_a1_Gsv*z^-1)

%% Continuous time controller design

% static feedforward controller
P_cv_kf = 0;                                %1 / k;

% Smith predictor with PI feedback controller
                                            % control parameter (dynamic compensation)
P_cv_Tw = 200e-3;                           % time constant closed loop Tw [ s ]
P_cv_Ti = P_p_T;                  
P_cv_Td = 0;
P_cv_kr = P_cv_Ti / (P_cv_Tw*P_p_k);
P_cv_Tt = P_p_Tt;
P_cv_delay = P_cv_Tt / P_dt;

% * Discrete time controller design *

% Deadbeat (general derived from plant)
P_cvdb_a1       = -P_p_a1_Gsv;              % denominator cof.
P_cvdb_b1       = P_p_b1_Gsv;               % numerator cof.

% Deadbeat - Dahlin's modification with given continuous time closed loop dynamic
% - PT1-Tt exp(-sTt)/(T0*s + 1)
% - exact discretization (zero order hold)
P_cvdbda_T0     = P_p_T*1.15;               % Time constant T0 [s]              
P_cvdbda_c1     = 1-exp(-P_dt/P_cvdbda_T0); % Conrol Parameter c1
P_cvdbda_c2     = exp(-P_dt/P_cvdbda_T0);   % Conrol Parameter c2


%% Parking Position Controller
P_cx_k = 2.7; % [ m/s / m ]
P_cx_umax = 0.5; % max. speed, control signal saturation [ m/s ]
P_cx_umin = -P_cx_umax;

%% Lateral Controller
Tw = 300e-3; % [ s ]
P_cy_kpsi = 2 * P_p_l / Tw;
P_cy_ky = P_p_l / Tw^2;
P_cy_v_min = 0.15; % minimum speed for state control [ m/s ]
P_cy_delta_Tt = P_p_Tt; % lookahead time [ s ]

%% Leading car detection
P_acc_lead_eymax = P_p_c_2 + 15e-3; % lateral deviation to detect leading car [ m ]
P_acc_self_eymax = P_p_c_2; % maximum allowed lateral deviation [ m ]
P_acc_self_kappamax = 12; % maximum allowed path curvature: tan(P_p_delta_max) / P_p_l = 11.2 [ 1/m ]

%% Adaptive Cruise Control
P_acc_Tt = 1.5 * P_p_Tt;
P_acc_amax_brake = 0.2 * P_p_k / P_p_T;
P_acc_xmin_anycase = P_p_c_1 + 15e-3; % min. distance in any case [ m ]
P_acc_xmin_accdeac = 50e-3; % min. ACC deactivation distance to lead vehicle [ m ]
P_acc_vlead_scale = 0.98; % reduced ego speed comparte to leading speed [ 1 ]

%% ACC test
P_acc_test1_vmin = 0.2;
P_acc_test1_vmax = 0.4;
P_acc_test2_vmin = 0.1;
P_acc_test2_vmax = 0.5;
P_acc_test_dt = 10; % [ s ]

%% Race
P_r_vmax = 0.4; % [ m/s ]
P_r_lane_init = uint32(1); % initial lane
P_r_passing_mindist = -P_p_c_1 * 3; % minimal distance to other car to switch back to right lane