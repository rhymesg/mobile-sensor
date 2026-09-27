

ts = 15;
v = 3;
rho = 15/pi;
n = 50;

num = 100;
N = zeros(1,num);
the = zeros(1,num);
the2 = zeros(1,num);

% k1 = 20;
% k2 = 64;

k1 = 0.0284;
k2 = 43.9651;

for k = 1:1:num;
    
    N(k) = 3 + (20 - 3)/num*(k-1);
 
    tw = ts - 4*rho/v - k2/v/(N(k)-1)^2;
    the(k) = 1/(N(k)-1) * exp(-tw/k1/n);
%     the2(k) = 1/(N(k)-1) * exp(-tw/5);

end

figure;
hold on;
plot(N, the, 'k', 'linewidth', 1.5);
% plot(N, the2, 'k', 'linewidth', 1.5);
axis([2 18 0 0.012]);
xlabel('N');
ylabel('the');
