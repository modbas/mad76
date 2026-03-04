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


function [ fd, hd, Ad, Bd, Cd, Gd ] = ...
    mbc_ekf_design(fc, hc, Gc, xc, xmc, u, pc, param, nTt, ...
    filename)
%% mbc_ekf_design discretizes fc function vector by 
%% backward differences

% get sizes
nc = length(xc);
mc = length(u);
Gc_size = size(Gc);
zc = Gc_size(2);

% args of continuous time model
cmargs = num2cell([xmc ; u]);
cargs = num2cell([xc ; u]);

% full model including dead time
n = nc + pc*nTt;
x = sym('x', [ n 1 ]);
xm = sym('xm', [ n 1 ]);

% backward differences
Ts = sym('Ts', 'real');
sol = solve(xc == xmc + Ts * fc, xc);
fdarr = sym('fdarr', [ n 1 ], 'real');
fields = fieldnames(sol);
for i = 1:length(fields)
    fdarr(i) = sol.(fields{i});
end

% dead time
hdarr = sym('hdarr', [ pc 1 ], 'real');
hcmarr = hc(cmargs{:}); % convert symfun to sym array
if nTt > 0
    for i = 1:pc
        fdarr(nc+i) = hcmarr(i);
        hdarr(i) = x(nc+(nTt-1)*pc+i);
    end
    for j = 1:nTt-1
        for i = 1:pc
            fdarr(nc+j*pc+i) = xm(nc+(j-1)*pc+i);
        end
    end
    Gc = [ Gc ; zeros(nTt*pc, zc) ];
else
    hdarr = hc(cargs{:});
end
fd([xm ; u ]) = simplify(fdarr);
hd([x ; u ]) = simplify(hdarr);

% Discrete-Time LTI (backward differences)
Ad = jacobian(fd, xm);
Bd = jacobian(fd, u);
Cd = jacobian(hd, x);
Gd = simplify(Ad * Ts * Gc);

%% Generate C code
% ccode(Ad, 'File', [ filename '_code_Ad.h' ]);
%ccode(Bd, 'File', [ filename '_code_Bd.h' ]);
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