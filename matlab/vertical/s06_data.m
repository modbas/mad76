%%
%% Mini-Auto-Drive
%%
%% System Parameters s06 for vertical architecture
%%
%% Copyright (C) 2017-2025, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

%% Add Path MODBAS CAR MATLAB Lib
addpath('../mbc');

%% Clear Workspace
clear all;

%% Solver Settings
P_dt = 25e-3; % Sampling time of controller and planner
P_display_dt = 50e-3; % Sampling time of display
P_task_dt = 2500e-6; % Task sampling time (required for time simulation)
P_sim_dt = P_task_dt; % Sampling time of simulation
 
%% Simulink Model Data Files
i01_data;
% d01_data;
% t02_race;

%% MAD76
d71_data;
t71_mad76_small;

%% Savitzky-Golay Filter
P_vfilter = single([ 0.0 0.0  ; 
                    1.0 -1.0 ...
                   ] / P_dt);

% P_vfilter = single([0.0 0.0 0.0 0.0 0.0 0.0 0.0 ; ...
%                     1.0 -1.0 0.0 0.0 0.0 0.0 0.0 ; ...
%                     0.5 0.0 -0.5 0.0 0.0 0.0 0.0 ; ...
%                     1.0500   -0.6500   -0.8500    0.450 0.0 0.0 0.0 ; ...
%                     0.771428571428572  -0.185714285714286  -0.571428571428573  -0.385714285714287   0.371428571428572 0.0 0.0 ; ...
%                     0.589285714285715  -0.003571428571429  -0.328571428571429  -0.385714285714287  -0.175000000000000   0.303571428571430 0.0  ; ...
%                     0.464285714285714   0.071428571428572  -0.178571428571428  -0.285714285714285  -0.250000000000000  -0.071428571428571 0.25 ...
%                     1.019841269841301  -0.48412698412700  -0.734126984127009  -0.285714285714292   0.305555555555571 0.484126984127005  -0.305555555555571
%                     ] / P_dt);
% fliplr(sgsdf(-6:0, 3, 1, 0)) % last row

%% Simulink data for track
P_track.size = single(track.size);
P_track.width = single(P_width);
P_track.right = convert2simulink(track.right, track.periodic);
P_track.left = convert2simulink(track.left, track.periodic);
P_track.center = convert2simulink(track.center, track.periodic);
P_track.lanes = [ ...
    convert2simulink(track.lanes(1), track.periodic), ...
    convert2simulink(track.lanes(2), track.periodic) ...
    ];

%% Init car display (carid 0)
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
% Init car display (carid 1)
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
% Init car display (carid 2)
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2);

%% auxiliary function to Simulink track data
function data = convert2simulink(path, periodic)
global SPLINE;
data.breakslen = uint32(length(path.points));
assert(data.breakslen <= SPLINE.Elements(2).Dimensions(2));
data.points = single(zeros(SPLINE.Elements(2).Dimensions)); 
data.points(:,1:length(path.points)) = single(path.points);
data.coefs = single(zeros(SPLINE.Elements(3).Dimensions));
data.coefs(1:length(path.pp.coefs),:) = single(path.pp.coefs);
data.segments = uint32(zeros(SPLINE.Elements(4).Dimensions));
data.periodic = periodic;
data.dx = single(path.dx);
end