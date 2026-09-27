% Single-target dispatch experiment; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.
clear;
clc;
close all;

%%
cx = 20; % center of C
cy = 0;
Rc = 40;

load('opt_sol_dispatch.mat');
length = 100;
for k = 1:1:length;
    
    dist = norm([x_res(k)-cx, y_res(k)-cy]);
    if ( dist < Rc)
        sind = k;
        break;
    end
end

dt = 18/length;
tau = 2*dt;

maxV = 3.10;
maxU = maxV/4;

J_init = eye(3);
vx(1) = 0;
vy(1) = 0;

the_a = -60*pi/180;
the_b = -78*pi/180;
len = 100;
J_f_best = 0;
for n = 1:1:len;
    angle = the_a + (the_b - the_a)/len * n;
    x(1) = cx + Rc*cos(angle);
    y(1) = cy + Rc*sin(angle);
    if (n == len)
        x(1) = x_res(sind);
        y(1) = y_res(sind);
    end
    z = 20;
    [gx gy J_dot] = control_grad([x(1) y(1) z], [xt_res(sind) yt_res(sind) 0]);
    ctr = [gx gy].*maxV/norm([gx gy]);
    vx(1) = ctr(1);
    vy(1) = ctr(2);
    
    J = J_init;
    J_rt(1) = -log(det(J_init));
    range = 0;
    for k = 1:1:length-sind;
    
        [gx gy J_dot] = control_grad([x(k) y(k) z], [xt_res(k+sind-1) yt_res(k+sind-1) 0]);
        ctr = [gx gy].*maxV/norm([gx gy]);
        ux = ctr(1);
        uy = ctr(2);

        J = J + J_dot*dt;
        J_rt(k+1) = -log(det(J));

        x(k+1) = x(k) + vx(k)*dt;
        y(k+1) = y(k) + vy(k)*dt;
        vx_dot = -(vx(k) - ux)/tau;
        vy_dot = -(vy(k) - uy)/tau;

        vx_dot = satur(vx_dot, -maxU, maxU);
        vy_dot = satur(vy_dot, -maxU, maxU);

        vx(k+1) = vx(k) + vx_dot*dt;
        vy(k+1) = vy(k) + vy_dot*dt;
        
        range = range + norm([x(k+1)-x(k), y(k+1)-y(k)]);
    end
    J_f(n) = J_rt(k+1);
    if (J_f(n) < J_f_best)
        x_best = x;
        y_best = y;
        n_best = n;
        J_f_best = J_f(n);
    end
    
    if (n == len)
        x_optinit = x;
        y_optinit = y;
    end
   
end

J = J_init;
range_opt = 0;
for k = sind:1:length;
    [gx gy J_dot] = control_grad([x_res(k) y_res(k) z], [xt_res(k) yt_res(k) 0]);
    J = J + J_dot*dt;
    J_opt(k+1) = -log(det(J));
    if (k < length)
        range_opt = range_opt + norm([x_res(k+1)-x_res(k), y_res(k+1)-y_res(k)]);
    end
end

range
range_opt

a = the_a;
b = the_b;
for i = 1:1:100;
    ang = a + (b-a)*i/100;
    cirx(i) = cx + Rc*cos(ang);
    ciry(i) = cy + Rc*sin(ang);
end

figure(1);
plot(x_res(1:end), y_res(1:end), 'r', 'linewidth', 2); hold on;
plot(x_best, y_best, 'b--', 'linewidth', 2); hold on;
plot(x_optinit, y_optinit, 'k-.', 'linewidth', 2); hold on;
plot(xt_res, yt_res, 'k:', 'linewidth', 2); hold on;
plot(cirx, ciry, 'k');
axis ([0 52 -50 2]);
legend('Optimal', 'Proposed', 'Grad-at-opt', 'Target', 'C_b');
xlabel('x (m)');
ylabel('y (m)');

opt = exp(0.5*J_opt(end));
opt/exp(0.5*J_f_best)
opt/exp(0.5*J_f(len))
