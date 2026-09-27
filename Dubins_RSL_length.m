function [ L ] = Dubins_RSL_length( a, b, d )
% Normalized RSL path length; DOI: 10.1016/j.inffus.2018.01.010, Eqs. (31)-(32).
% See docs/algorithm.md, docs/running.md, and README.md#citation.

p = sqrt(d^2 - 2 + 2*cos(a-b) - 2*d*(sin(a) + sin(b) ) )
t = mod(a - atan2( cos(a) + cos(b), d - sin(a) - sin(b) ) + atan2(2, p) , 2*pi)
L = -a + b + 2*t + p

end

