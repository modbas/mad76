function [ y, yd, ydd, pidx ] = mbc_ppval(varargin)
%#codegen
% y = mbc_ppval(pp, x)
% y = mbc_ppval(ppbreaks, ppcoefs, x, pidx) embedded version of ppval.
% Used in Simulink code generated for the car.
%
% See also ppval.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle


lap = single(0);

if nargin == 2
    ppbreaks = varargin{1}.breaks;
    breakslen = length(ppbreaks);
    ppcoefs = varargin{1}.coefs;
    xarr = varargin{2};
else
    breakslen = varargin{1};
    ppbreaks = varargin{2};
    ppcoefs = varargin{3};
    xarr = varargin{4};
end
if nargin == 5
    pidx = int32(varargin{5});
else
    if xarr(1) > ppbreaks(breakslen)
        lap = single(1);
    end
    pidx = mbc_binsearch(breakslen, ppbreaks, xarr(1) - lap * ppbreaks(breakslen));
end

y = zeros(2, length(xarr), 'single');
yd = zeros(2, length(xarr), 'single');
ydd = zeros(2, length(xarr), 'single');

for i = 1:length(xarr)
    x = xarr(i) - lap * ppbreaks(breakslen);    
    if i > 1
        if x >= ppbreaks(pidx+1)
            pidx = pidx + 1;
        end
        if int32(pidx) >= int32(breakslen)
            lap = lap + 1;
            x = xarr(i) - lap * ppbreaks(breakslen);    
            pidx = int32(1);
        end
    end
    
    c = ppcoefs((2*pidx-1):(2*pidx), :);
    dx = x - ppbreaks(pidx);
    y(:,i) = c(:,1);
    for cidx = 2:length(c)
        y(:,i) = y(:,i) * dx + c(:,cidx);
    end
    yd(:,i) = (3 * c(:,1) * dx + 2 * c(:,2)) * dx + c(:,3);
    ydd(:,i) = 6 * c(:,1) * dx + 2 * c(:,2);
end
end