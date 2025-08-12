function flag = mbc_cmp(x1, x2)
% flag = mbc_cmp(x1, x2) compares 2 real numbers.
epsi = mbc_cmp_eps; % 1mm
flag = ((x1 >= x2 - epsi) && (x1 <= x2 + epsi));
end