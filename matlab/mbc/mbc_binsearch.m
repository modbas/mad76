function bidx = mbc_binsearch(breakslen, ppbreaks, x)
%#codegen
% bidx = mbc_binsearch(breaks, x) searches for closest match of x in breaks
%   breakslen - number of breaks
%   breaks - are the breaks of a piecewise polynomial
%   x - is the arc length
%   bidx - is the idx of the breaks element with
%          breaks(bidx) <= x < breaks(bidx+1)
% 
%   mbc_binsearch performs a binary search and is called by mbc_ppval.
%
% See also mbc_ppval.
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

bidx1 = int32(1);
bidx2 = int32(breakslen);
while bidx1 < bidx2 - 1
    bidx = int32(floor(single(bidx1 + bidx2) / 2));
    if x >= ppbreaks(bidx)
        bidx1 = bidx;
    elseif x < ppbreaks(bidx)
        bidx2 = bidx;
    end
end
bidx = bidx1;
end