function config = mbc_config()
% config = mbc_config() returns config data for MODBAS CAR.
%   config - struct containing config data
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle
config.track.debug = 0;
config.track.color = 'k';
config.track.centercolor = 'w';
config.track.centerwidth = 1;
config.track.centerstyle = '--';
config.track.bgcolor = zeros(1, 3);
config.track.bordercolor = 'w';
config.track.borderwidth = 2;
config.track.borderstyle = '-';
config.figure.bgcolor = 0.3 * ones(1, 3);
config.car.colors = { [1 0.3 0], [0.7 1 0], [0 0 1], [1 1 1] };
config.carwheel.color = 'r';
config.carwheel.w = 0.005;
config.carwheel.l = 0.03;
config.carvelocity.color = 'w';
config.refvelocity.color = 'r';
config.ir.color = 'r';
config.cartrace.size = 0.003;
config.cartrace.color = [ 1 1 1 1 ];
config.cartrace.length = 100;
config.cartrace.dt = 25e-3;
config.carerror.color = 'w';
config.obstacle.color = 'w';
end