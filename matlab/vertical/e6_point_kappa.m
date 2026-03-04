%%
%% Mini-Auto-Drive
%%
%% Extended Kalman-Filter Design
%% kinematic-only general object tracking
%%
%% Copyright (C) 2017-2026, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

clear variables;

% parameters
Ts = sym('Ts', 'real'); % sampling time [ s ] 
param = [ Ts ];

% x = [ s1 s2 psi v a kappa ]
nc = 6;
mc = 0;
pc = 2; % number of outputs
x = sym('x', [ nc, 1 ], 'real');
xm = sym('xm', [ nc, 1 ], 'real');
% u = [  ]
u = sym('u', [ mc, 1 ], 'real');

% Continuous-Time
% \dot{x} = f(x, u)
% y = h(x, u)
fc([ x ; u ]) = [ ...
    x(4) * cos(x(3)) ; ... % x(1) = pos. s1 [ m ]
    x(4) * sin(x(3)) ; ... % x(2) = pos. s2 [ m ]
    x(4) * x(6) ; ...      % x(3) = velocity angle gamms [ rad ]
    x(5) ; ...             % x(4) = speed [ m/s ]
    0 ; ...                % x(5) = acceleration [ m/s^2 ]
    0 ];                   % x(6) = curvature [ 1/m ]
hc([ x ; u ]) = [ ...
    x(1) ; ...
    x(2) ; ...
    % x(3) ...
    ];
Gc = [ ...
    0 0 ; ...
    0 0 ; ...
    0 0 ; ...
    0 0 ; ...
    1 0 ; ...
    0 1 ; ...
    ];

% Backward Differences
[ fd1, hd1, A1, B1, C1, G1 ] = mbc_ekf_design(...
    fc, hc, Gc, x, xm, u, pc, param, 0, ...
    'e6');

% Circle Drive
% fd2([ x ; u ]) = [ ...
%     xm(1) + x(4)/x(5)*(-sin(xm(3)) + sin(x(3))) ; ... % pos. sr1
%     xm(2) + x(4)/x(5)*(cos(xm(3)) - cos(x(3))) ; ... % pos. sr2 
%     xm(3) + Ts * x(5) ; ... % yaw angle psi
%   (vmax/(vmax+(2*f*amax*dt)))*xm(4) ; ... % speed 
%     xm(5) ... % yaw rate
%     ];
% hd2([ x ; u ]) = [ ...
%     x(1) ; ...
%     x(2) ; ...
%     x(3) ; ...
%     ];
% [ fd2, A2, B2, C2, G2 ] = mbc_ekf_design_discrete(...
%     fd2, hd2, Gc, x, xm, u, param, ...
%     'e5circle');

