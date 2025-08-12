function track = mbc_circle_create(track, r, rad, w)
% track = mbc_circle_create(track, r, rad, w) adds a circular arc 
% as a new segment to track
% 
%   track - existing track created by mbc_track_create.
%           The return value track contains the original track plus
%           the new segment.
%   r - radius of circle [ m ]
%   rad - arc length [ rad ]
%   w - track width [ m ]
%
%   If rad > 0 then mbc_circle_create creates a left turn.
%   If rad < 0 then mbc_circle_create creates a right turn.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle
cnt = mbc_track_get_cnt(track);
p = track.points{cnt+1};
xe = rad * r; % path length
center1 = p.s1 + sign(xe) * r * cos(p.psi + pi/2);
center2 = p.s2 + sign(xe) * r * sin(p.psi + pi/2);
track.points{cnt+2} = ...
    struct('s1', center1 - sign(xe) * r * cos(p.psi + pi/2 + rad), ...
    's2', center2 - sign(xe) * r * sin(p.psi + pi/2 + rad), ...
    'psi', p.psi + rad, ...
    'x', p.x + abs(xe));
track.tracks{cnt+1} = struct('type', 'circle', ...
    'r', r, 'xe', xe, 'w', w);
end