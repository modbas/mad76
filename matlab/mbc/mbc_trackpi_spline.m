function path = mbc_trackpi_spline(points, dx)
%% creates spline from points
%
%   points - array of 2 rows for s1 and s2 coordinates

if ~exist('dx', 'var')
    dx = 0;
end

if mbc_cmp(points(1,1), points(1,end)) && mbc_cmp(points(2,1), points(2,end))
    % ensure that first and last points are identical
    points(:,end) = points(:,1);
    diffp2 = (diff(points') .^ 2)';
    diffx = sqrt(diffp2(1,:) + diffp2(2,:));
    diffx(end+1) = diffx(1); % add first element to end so that x has size of points (works for periodic splines)
    x = cumtrapz(diffx);
    pp = csape(x, points, 'periodic');
    len = length(x);
    if dx > 0
        len = ceil(x(end) / dx);
    end
    xnew = linspace(0, x(end), len);
    pointsnew = ppval(pp, xnew);
    ppnew = csape(xnew, pointsnew, 'periodic');
    path = struct('pp', ppnew, ...
        'ppd', fnder(ppnew), ...
        'ppdd', fnder(ppnew, 2), ...
        'points', [ xnew ; pointsnew ]);
else
    % error('Start end end points to not match for periodic spline interpolation');
    % otherwise create non-periodic splines
    diffp2 = (diff(points') .^ 2)';
    diffx = sqrt(diffp2(1,:) + diffp2(2,:));
    x = [0, cumsum(diffx)];
    pp = csape(x, points, 'variational');
    len = length(x);
    if dx > 0
        len = ceil(x(end) / dx);
    end
    xnew = linspace(0, x(end), len);
    pointsnew = ppval(pp, xnew);
    ppnew = csape(xnew, pointsnew, 'variational');
    path = struct('pp', ppnew, ...
        'ppd', fnder(ppnew), ...
        'ppdd', fnder(ppnew, 2), ...
        'points', [ xnew ; pointsnew ]);
end
end
