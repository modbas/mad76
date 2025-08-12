function splin = mbc_spline_create(track, dx, alpha)
% splin = mbc_spline_create(track, dx, alpha) interpolates track by
% spline.
%
%   track - track including path segments
%   dx - step size of arc length for interpolation point calculation [ m ]
%   alpha - position on track [ -1; 1 ]
%   splin - struct containing the track spline, its first and second
%          derivative.
%
%   Each spline in this struct is a two-dimensional piecewise polynomial.
%   One polynomial for the x-position and on for the y-position.
%   All polynomials are parameterized by the arc length.
%
%   If alpha == 0 then the right line points are generated.
%   If alpha == 0.5 then the center line points are generated.
%   If alpha == 1 then the left line points are generated.
%
% mbc_spline_create uses csape (part of the Curve Fitting Toolbox)
% to create a periodic spline for periodic tracks. If no Curve Fitting
% Toolbox is available spline is used instead of csape. But spline
% is not really useful for periodic tracks.
%
% See also csape, spline.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

points = [];
x = 0;
for idx = 1:mbc_track_get_cnt(track)
    switch track.tracks{idx}.type
        case 'straight'
            newpoints = mbc_straight_get_points(track, idx, x, dx, alpha);
        case 'circle'
            newpoints = mbc_circle_get_points(track, idx, x, dx, alpha);
        case 'clothoid'
            newpoints = mbc_clothoid_get_points(track, idx, x, dx, alpha);
        otherwise
            error('track.type not supported');
    end
    x = newpoints(1, end) + dx;
    points = [ points newpoints ];
end
if track.periodic
    % We have a circular track.
    % Ensure last point equals to first point.
    points =  [ points(:, 1:end-1) [ points(1, end); points(2:3, 1) ] ];
    if exist('csape', 'file')
        pp = csape(points(1, :), points(2:3, :), 'periodic');
        % if exist('spapi', 'file')
        %     pp = fn2fm(spapi(5, points(1, :), points(2:3, :)), 'pp');
    else
        pp = spline(points(1, :), points(2:3, :));
    end
else
    pp = spline(points(1, :), points(2:3, :));
end

splin = struct('pp', pp, 'ppd', fnder(pp, 1), 'ppdd', fnder(pp, 2), ...
    'points', points); %, 'dx', dx);
end