%%
%% Mini-Auto-Drive
%%
%% Extended Kalman-Filter Design
%%
% Copyright (c) 2020 Frank Traenkle
% http://www.modbas.de
%
% This file is part of Mini-Auto-Drive.
%
% Mini-Auto-Drive is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Mini-Auto-Drive is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.

function [ fd, Ad, Bd, Cd, Gd ] = ...
    mbc_ekf_design_discrete(fd, hd, Gc, x, xm, u, param, ...
    filename)
%% generates MATLAB code for discrete EKF model

Ts = sym('Ts', 'real');
n = size(x, 1);

% substitute x by xm
sol = solve(x == fd, x);
fdarr = sym('fdarr', [ n 1 ], 'real');
fields = fieldnames(sol);
for i = 1:length(fields)
    fdarr(i) = sol.(fields{i});
end
fd([xm ; u ]) = simplify(fdarr);

% Linear LTI
Ad = jacobian(fd, xm);
Bd = jacobian(fd, u);
Cd = jacobian(hd, x);
Gd = simplify(Ad * Ts * Gc);

%% Generate C code
% ccode(Ad, 'File', [ filename '_code_Ad.h' ]);
% ccode(Bd, 'File', [ filename '_code_Bd.h' ]);
% ccode(Cd, 'File', [ filename '_code_Cd.h' ]);
% ccode(Gd, 'File', [ filename '_code_Gd.h' ]);
% ccode(fd, 'File', [ filename '_code_fd.h' ]);
% ccode(hd, 'File', [ filename '_code_hd.h' ]);

%% Generate MATLAB code
matlabFunction(Ad, Bd, Cd, Gd, 'File', [ filename '_linsys' ], ...
    'Vars', { x, xm, u, param }, ...
    'Comment', '#codegen');
matlabFunction(fd, 'File', [ filename '_nonlinsys_f' ], ...
    'Vars', { xm, u, param }, ...
    'Comment', '#codegen');
matlabFunction(hd, 'File', [ filename '_nonlinsys_h' ], ...
    'Vars', { x, u, param }, ...
    'Comment', '#codegen');
end