function this = mbc_track_create(s1, s2, psi, size)
% this = mbc_track_create(s1, s2, psi) creates a new empty track.
%   s1 - initial x-coordinate
%   s2 - initial y-coordinate
%   psi - initial yaw angle
%   this - the new track
%
% See also mbc_straight_create, mbc_circle_create
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

if ~exist('size', 'var')
    size = [];
end
this = struct('points', cell(1), ...
    'tracks', cell(1), ...
    'obstacles', cell(1), ...
    'size', size, ...
    'type', 24);
this.points{1} = struct('s1', s1, ...
    's2', s2, 'psi', psi, 'x', 0);
end