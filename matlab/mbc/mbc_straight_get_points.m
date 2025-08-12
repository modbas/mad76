function [ points ] = mbc_straight_get_points(track, idx, xstart, dx, alpha)
% points = mbc_straight_get_points(track, idx, sstart, ds, alpha)
% computes the accurate line points on the track segment.
%
%   track - track object
%   idx - the index of the track segment
%   xstart - the arc length of the segment start positon [ m ]
%   dx - the arc distance of two neighbor points [ m ]
%   alpha - the line position on the road [ 0 ; 1 ]
%   points - the points as a 3 x n matrix
%
%   points(1, :) is the arc length at each point
%   points(2, :) is the x position
%   points(3, :) is the y position
%
%   If alpha == 0 then the right line points are generated.
%   If alpha == 0.5 then the center line points are generated.
%   If alpha == 1 then the left line points are generated.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle
if nargin < 4
    alpha = 0.5; % return center points
end
p = track.points{idx};
t = track.tracks{idx};
cpsi = cos(p.psi);
spsi = sin(p.psi);
idx = 0:floor((t.xe - (xstart - p.x) + mbc_cmp_eps)/dx);
x = (xstart - p.x) + dx * idx;
points = [ p.x + x ; ...
    p.s1 + cpsi * x + (alpha - 0.5) * t.w * cos(p.psi + pi/2) ; ...
    p.s2 + spsi * x + (alpha - 0.5) * t.w * sin(p.psi + pi/2) ];
end