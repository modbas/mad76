%%
%% MAD76
%%
%% creates spline from points recorded from simulation of agent
%%
%% Copyright (C) 2017-2025, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.6de, Hochschule Heilbronn, Germany
%%


ds1 = 1.25e-3; % assume 2mm resolution, since 25ms * 0.5 m/s = 1.25mm
ds2 = 10e-3; % find point on starting line
dx = 20e-3; % resolution of resulting spline

p = logsout.get('s').Values.Data;

% search for starting point
s0 = [ 0.4 , 0.373 ];

idx1list = find(abs(p(:,1) - s0(1)) <= ds1 & abs(p(:,2) - s0(2)) <= ds2);
idx1 = idx1list(1);
s1 = p(idx1, :);

idx2 = find(abs(p(:,1) - s1(1)) <= ds1 & abs(p(:,2) - s1(2)) <= ds2);
idx2list = idx2(idx2 > idx1(1) + 10);
idx2 = idx2list(1);

sopt = p(idx1:idx2, :);

figure(2); clf;
plot(sopt(:, 1), sopt(:, 2), 'b.');
grid on;
hold on;
axis equal;

% ensure cyclic spline
sopt(end, :) = sopt(1, :);
sopt = sopt';

path = mbc_trackpi_spline(sopt, dx);
%fnplt(path.pp, 'r.');
plot(path.points(2,:), path.points(3,:), 'r.');

save('t71_mad76_sopt', 'sopt');
