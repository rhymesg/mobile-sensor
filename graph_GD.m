

ts = 10;
n = 20;

num = 100;
x = zeros(1,num);
y_smalltheta = zeros(1,num);
y_largetheta = zeros(1,num);
y_w = zeros(1,num);
x2 = zeros(1,num);

k1 = 0.15;
sthe = 0.001;
lthe = 0.003;
k2 = 10;

for k = 1:1:num;
    
    x(k) = 1 + (20 - 1)/num*(k-1);
    x2(k) = x(k)/10;
    
    y_smalltheta(k) = k1*n*log2(1/(n*sthe*x(k)));
    y_largetheta(k) = k1*n*log2(1/(n*lthe*x(k)));
    y_w(k) = ts - k2/x(k);
    
    y_theta(k) = k1/ts/x2(k) + k2;
end

figure;
hold on;
plot(x, y_smalltheta, 'k--', 'linewidth', 1.5);
plot(x, y_w, 'k', 'linewidth', 1);
plot(0:1:20, ts*ones(1,21), 'k:', 'linewidth', 1);
plot(x, y_largetheta, 'k--', 'linewidth', 1.5);
axis([0 20 0 15]);
legend('aaaaaaaaaaaaaaaaaaaaaa', '            ', '            ');
xlabel('N');
ylabel('t_w');
% 
% figure;
% plot(x2, y_theta, 'k', 'linewidth', 1.5);
% xlabel('Predefined Error (rad)');
% ylabel('N');
% axis([0 2 0 50]);