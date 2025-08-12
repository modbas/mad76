function ret = mbc_car_display(t, s1, s2, psi, beta, v, deltan, ...
    ws1, ws2, wpsi, wv, carid, init)
% ret = mbc_car_display(t, x, y, psi, delta, vb, gpsx, gpsy, ex, ey)
% displays car on road track. This function can be called from Simulink.
%   t - time
%   x - x-position
%   y - y-position
%   psi - yaw angle
%   delta - steering angle
%   vb - belt speed (only relevant for belt environment)
%   gpsx - GPS x position
%   gpsy - GPS y position
%   ex - state estimator output for x-position
%   ey - state estimator output for y-position
%   ret - always 0
%
% MODBAS CAR mbc
% Copyright (C) 2015, Frank Traenkle

global P_p_l P_p_rear_1 P_p_c_1 P_p_c_2 P_p_delta_max;
persistent config carstruct;
    
if nargin() < 13
    init = false;
end

if isempty(config)
    config = mbc_config();
end

if gcf().Number ~= 1
    figure(1);
end

if isempty(carstruct) || length(carstruct) < carid+1 || init
    figure(1);
    carstruct{carid+1}.car = patch([ 0; P_p_c_1; P_p_c_1; 0 ], [ 0; 0; P_p_c_2; P_p_c_2 ], ...
        [ 0; 0; 0; 0 ], ...
        'FaceColor', config.car.colors{carid+1}, 'EdgeColor', config.car.colors{carid+1});
    carstruct{carid+1}.wheel = patch([ 0; config.carwheel.w; config.carwheel.w; 0 ], ...
        [ 0; 0; config.carwheel.l; config.carwheel.l ], ...
        [ 0; 0; 0; 0 ], ...
        'FaceColor', config.carwheel.color, 'EdgeColor', config.carwheel.color);        
    carstruct{carid+1}.velocity = patch([ 0; 0; 0; 0 ], ...
        [ 0; 0; 0; 0 ], ...
        [ 0; 0; 0; 0 ], ...
        'FaceColor', config.carvelocity.color, 'EdgeColor', config.carvelocity.color);        
    carstruct{carid+1}.refvelocity = patch([ 0; 0; 0; 0 ], ...
        [ 0; 0; 0; 0 ], ...
        [ 0; 0; 0; 0 ], ...
        'FaceColor', config.refvelocity.color, 'EdgeColor', config.refvelocity.color);        
    carstruct{carid+1}.trace = [];
elseif init
    carstruct{carid+1}.trace.tout = 0;
    carstruct{carid+1}.trace.idx = 1;
    for idx=1:config.cartrace.length
        if carstruct{carid+1}.trace.trace(idx) ~= 0
            x = 0;
            y = 0;
            color = [ 0 0 0 ];
            set(carstruct{carid+1}.trace.trace(idx), 'XData', [ x - config.cartrace.size; x + config.cartrace.size; ...
                    x + config.cartrace.size; x - config.cartrace.size ], ...
                    'YData', [ y - config.cartrace.size; y - config.cartrace.size; ...
                    y + config.cartrace.size; y + config.cartrace.size ], ...
                    'FaceColor', color, 'EdgeColor', color);
        end
    end
end

car = carstruct{carid+1}.car;
wheel = carstruct{carid+1}.wheel;
velocity = carstruct{carid+1}.velocity;
refvelocity = carstruct{carid+1}.refvelocity;

%% Car
delta = deltan * P_p_delta_max;
s = sin(psi);
c = cos(psi);
w = 0.5*P_p_c_2;
xr = s1 + P_p_rear_1 * c;
yr = s2 + P_p_rear_1 * s;
xf = xr + P_p_c_1 * c;
yf = yr + P_p_c_1 * s;
set(car, 'XData', [ xr+w*s; xf+w*s; xf-w*s; xr-w*s ]);
set(car, 'YData', [ yr-w*c; yf-w*c; yf+w*c; yr+w*c ]);

%% Front wheel
l = 0.5*config.carwheel.l;
w = 0.5*config.carwheel.w;
xf = s1 + P_p_l * c;
yf = s2 + P_p_l * s;
s = sin(psi+delta);
c = cos(psi+delta);
set(wheel, 'XData', [ xf+w*s-l*c; xf+w*s+l*c; xf-w*s+l*c; xf-w*s-l*c ]);
set(wheel, 'YData', [ yf-w*c-l*s; yf-w*c+l*s; yf+w*c+l*s; yf+w*c-l*s ]);

%% Velocity vector
s = sin(psi);
c = cos(psi);
l = 0.5 * v;
set(velocity, 'XData', [ s1; s1+l*c; s1+l*c; s1 ]);
set(velocity, 'YData', [ s2; s2+l*s; s2+l*s; s2 ]);

%% Reference velocity vector
s = sin(wpsi);
c = cos(wpsi);
l = 0.5 * wv;
set(refvelocity, 'XData', [ ws1; ws1+l*c; ws1+l*c; ws1 ]);
set(refvelocity, 'YData', [ ws2; ws2+l*s; ws2+l*s; ws2 ]);

%% Car trace
carstruct{carid+1}.trace = mbc_car_trace(t, s1, s2, v, config.cartrace, carstruct{carid+1}.trace, config.car.colors{carid+1});

ret = 0;
end

function trace = mbc_car_trace(t, x, y, v, traceconfig, traceold, color)
if isempty(traceold)
    trace.trace = zeros(traceconfig.length, 1);
    trace.idx = 1;
    trace.tout = traceconfig.dt;
else
    trace.trace = traceold.trace;
    trace.idx = traceold.idx;
    trace.tout = traceold.tout;
end
if (traceconfig.size > 0) && (t >= trace.tout)
    % vnormalized = min([abs(v) / 2.51 , 1 ]);
    % color = [ 1-vnormalized, vnormalized, 0 ];
    %% display trace
    if trace.trace(trace.idx) == 0
        trace.trace(trace.idx) = patch( [ x - traceconfig.size; x + traceconfig.size; ...
            x + traceconfig.size; x - traceconfig.size ], ...
          [ y - traceconfig.size; y - traceconfig.size; ...
            y + traceconfig.size; y + traceconfig.size ], ...
          [ 0; 0; 0; 0 ], ...
        'FaceColor', color, 'EdgeColor', color);
    else
        set(trace.trace(trace.idx), 'XData', [ x - traceconfig.size; x + traceconfig.size; ...
            x + traceconfig.size; x - traceconfig.size ], ...
            'YData', [ y - traceconfig.size; y - traceconfig.size; ...
            y + traceconfig.size; y + traceconfig.size ], ...
            'FaceColor', color, 'EdgeColor', color);
    end
    % for idx = 1:(traceconfig.length-1)
    %     oldidx = trace.idx - idx;
    %     if oldidx < 1
    %         oldidx = oldidx + traceconfig.length;
    %     end
    %     old = trace.trace(oldidx);
    %     if old ~= 0
    %         set(old, 'EdgeColor', traceconfig.color * (traceconfig.length - 0.6 * idx) / traceconfig.length);
    %     end
    % end
    if trace.idx >= traceconfig.length
        trace.idx = 1;
    else
        trace.idx = trace.idx + 1;
    end
    trace.tout =  trace.tout + traceconfig.dt;
end
end