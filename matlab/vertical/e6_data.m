%%
%% Mini-Auto-Drive
%%
%% Extended Kalman-Filter e6 data
%%
%% Copyright (C) 2019-2026, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%


%% initial conditions and parameters
P_e_n = 6;
P_e_x0 = zeros(P_e_n, 1);
P_e_x0(3) = 0; %pi;
P_e_P0 = eye(P_e_n, P_e_n);
P_e_param = P_dt;

%% stochastic input noise
adot_std = 50; % standard deviation of da(t)/dt [ m/s^3 ]
kappadot_std = 50; % standard deviation of dkappa(t)/dt [ 1/(m*s) ]
P_e_Qu = diag([ adot_std^2 , kappadot_std^2 ]);

%% output noise
s1_std = 3e-3; % standard deviation of s1 [ m ]
s2_std = s1_std; % standard deviation of s2 [ m ]
psi_std = deg2rad(1); % standard deviation of psi [ rad ]
P_e_R =  diag([ s1_std^2 , s2_std^2]); % ,  psi_std^2]);
