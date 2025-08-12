function cnt = mbc_track_get_cnt(track)
% cnt = mbc_track_get_cnt(track) retrieves the number of segments of
% a track.
%
%   track - the track
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

cnt = length(track.tracks);
end