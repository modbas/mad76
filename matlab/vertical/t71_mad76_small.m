%%
%% MAD76
%%
%% Small track (100x50 cm)
%%
%% Copyright (C) 2017-2024, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.6de, Hochschule Heilbronn, Germany
%%

addpath('../mbc');
totalSize = [ -0.1, 0.82, 0.0, 0.5 ];

m = load('t71_mad76_params.mat');
leftPoints = [ m.sl1 ; m.sl2 ];
rightPoints = [ m.sr1 ; m.sr2 ];

mopt = load('t71_mad76_sopt.mat');


%% create and display track
track = mbc_trackpi_create(rightPoints, leftPoints, mopt.sopt, totalSize);
mbc_trackpi_display(track, NaN);

P_width = 230e-3; % [ m ] %mean(sqrt(sum((leftPoints - rightPoints).^2, 1)));

%% display boundaries for RL training
config = mbc_config();
if config.track.debug
    ey = 0.9;
    x = track.center.points(1, :);
    s = track.center.points(2:3, :);
    sd = mbc_ppval(track.center.ppd, x);
    psi = atan2(sd(2, :), sd(1, :));
    sl = s + 0.5 * P_width * ey * [ -sin(psi) ; cos(psi)];
    plot(sl(1, :), sl(2, :), 'yx');
    sr = s + 0.5 * P_width * ey * [ sin(psi) ; -cos(psi)];
    plot(sr(1, :), sr(2, :), 'yx');
end

