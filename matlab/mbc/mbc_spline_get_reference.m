function [ w, widx ]  = mbc_spline_get_reference(s, breakslen, points, ppcoefs, ...
    periodic, vref, Tt, irangemin, irangemax)
%#codegen
% w  = mbc_spline_get_reference(s, breakslen, points, ppcoefs, ...
%      periodic, vref, Tt) calculates the reference signal for
%      path following control.
%
%   s - current car position: vector with x- and y-coordinates
%   breakslen - number of breaks
%   points - interpolation points on spline. 3 x n matrix.
%            points(1,:) is the discrete arc length.
%            points(2,:) are the x-coordinates (s1).
%            points(3,:) are the y-coordinates (s2).
%   ppcoefs - piecewise polynomial coefficients of path
%   periodic - true if circular track, false otherwise
%   vref - speed for lookahead
%   Tt - steering delay for lookahead of kappa
%   w - reference signal: [ arc length x, orientation angle psi [rad],
%       s1, s2, curvature kappa ]
%   widx - index nearest point in spline
%
% See also mbc_spline_get_nearest, mbc_ppval.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

if breakslen > 0
    if nargin < 8
        irangemin = int32(1);
        irangemax = int32(breakslen);
    end
    [ wx, imin1 ] = mbc_spline_get_nearest(breakslen, points, ppcoefs, periodic, s, irangemin, irangemax);
    [ wsv, wsdv, wsddv, widx ] = mbc_ppval(breakslen, points(1,:), ppcoefs, wx, imin1);
    ws1 = wsv(1);
    ws2 = wsv(2);
    wpsi = atan2(wsdv(2), wsdv(1));

    %% lookahead
    if vref ~= 0
        wxlookahead = wx + vref * Tt;
        if periodic == true
            if wxlookahead > points(1, breakslen)
                wxlookahead = wxlookahead - points(1, breakslen);
            elseif wx < 0
                wxlookahead = wxlookahead + points(1, breakslen);
            end
        end
        [ wsv, wsdv, wsddv, widx ] = mbc_ppval(breakslen, points(1,:), ppcoefs, wxlookahead);
    end
%     % time derivatives
    v = sqrt(wsdv(1).^2 + wsdv(2).^2);
    if v > 0
        wkappasign = wsdv(1) * wsddv(2) - wsddv(1) * wsdv(2);
        wkappa = wkappasign / (v^3);
    else
        wkappa = single(0);
    end
%    if wkappasign < 0
%        wkappa = -wkappa;
%    end
    w = [ wx; wpsi; ws1; ws2; wkappa ];
else
    w = single([ 0; 0; 0; 0; 0 ]);
    widx = int32(-1);
end
end