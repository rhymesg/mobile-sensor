clc;
clear;

t_max = 100;
tol = 0.005;

x = 0;
for t = 1:1:t_max;
    
    r = 2.4*1/t;
    
    x = x - 0.2*r*2*(x-1.1312345)
    
    if (abs(x - 1.1312345) < tol)
        t
        break;
    end
end

k = t / (log(1/tol))
