function track = mbc_straight_create(track, xe, w)
% track = mbc_straight_create(track, xe, w) adds a straight line 
% as a new segment to track
% 
%   track - existing track created by mbc_track_create.
%           The return value track contains the original track plus
%           the new segment.
%   xe - arc length of straight line [ m ]
%   w - track width [ m ]
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle
cnt = mbc_track_get_cnt(track);
p = track.points{cnt+1};
track.points{cnt+2} = ...
    struct('s1', p.s1 + xe * cos(p.psi), ...
    's2', p.s2 + xe * sin(p.psi), ...
    'psi', p.psi, ...
    'x', p.x + xe);
track.tracks{cnt+1} = struct('type', 'straight', ...
    'xe', xe, 'w', w);
end