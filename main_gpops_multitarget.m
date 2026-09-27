% Multitarget optimal-control setup; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

clear all;

t0 = 0;
tm = 10; % operation time
ts_min = 15;
ts_max = 15;
OC = [20 0]; % center of operational area
RC = 30; % radius of operational area
alt = 20; % altitude of UA
sig_R = [0.07 0.07 0.1]; % standard deviation of sensor noise
V = 3; % speed of UA
L = 3; % number of targets
multisensor = 1;

xt_init = [25, -5, ...
          30, -15, ...
          40, 15,];

xt_err = [0, 0, ...
          0, 0, ...
          0, 0,];

xt_dot = [-1, 0.5, ...
         -0.5, 1, ...
          -0.1, 0.1];
      
xt_dot_err = [0, 0, ...
              0, 0, ...
              0, 0,];
%           
% xt_err = 2.0*randn(1,6);
% xt_dot_err = 0.1*randn(1,6);

% xt_err = [-0.887043143828039,0.741377097305350,1.39220787088793,2.47390028937627,0.503919242690866,-0.822480553830570];
% xt_dot_err = [0.0200981870646343,-0.100705252750908,-0.0613171560234073,-0.0658963390849518,-0.0833229327511956,0.0378179296887144];

J0 = mat2vec(eye(3,3), 'row');
Jf = mat2vec(500*eye(3,3), 'row');

x0 = [60, -60, pi*2/4, xt_init + xt_err, J0, J0, J0];
xf = [20, 0, pi*2/4, zeros(1,6), Jf, Jf, Jf];
xMin = [-100, -100, 0, -50*ones(1,6), -1500*ones(1,9*L)];
xMax = -xMin;
xMax(3) = 2*pi;
uMin = -pi/5;
uMax = pi/5;

auxdata.OC = OC;
auxdata.RC = RC;
auxdata.tm = tm;
auxdata.alt = alt;
auxdata.sig_R = sig_R;
auxdata.V = V;
auxdata.L = L;
auxdata.xt_dot = xt_dot + xt_dot_err;
auxdata.multisensor = multisensor;

bounds.phase(1).initialtime.lower = t0;
bounds.phase(1).initialtime.upper = t0;
bounds.phase(1).initialstate.lower = x0;
bounds.phase(1).initialstate.upper = x0;
bounds.phase(1).finaltime.lower = ts_min;
bounds.phase(1).finaltime.upper = ts_max;
bounds.phase(1).state.lower = xMin;
bounds.phase(1).state.upper = xMax;
bounds.phase(1).control.lower = uMin;
bounds.phase(1).control.upper = uMax;
bounds.phase(1).finalstate.lower = xMin;
bounds.phase(1).finalstate.upper = xMax;
bounds.eventgroup(1).lower = zeros(1,length(x0)+2);
bounds.eventgroup(1).upper = zeros(1,length(x0)+2);

guess.phase(1).time = [t0; (ts_min+ts_max)/2];
xg = [OC(1)+RC*cos(pi/4), OC(2)-RC*sin(pi/4), x0(3:end)];
if (multisensor == 1)
     JJ = [193.752104689370,-219.391557166970,297.891444463020,-219.391557166955,318.999543219824,-328.974122716990,297.891444462910,-328.974122717322,500.505990415076,245.353504910639,-314.152760765991,259.433162865007,-314.152760766185,468.010562308579,-355.388049744855,259.433162864855,-355.388049744569,288.057537774207,15.7769957502721,45.1598916763546,-42.3010120481593,45.1598916763289,476.945314948051,-446.303103684777,-42.3010120482055,-446.303103684696,523.317669784299];
     xg = [OC(1)+RC*cos(pi/4), OC(2)-RC*sin(pi/4), xf(3:9), JJ];
end
guess.phase(1).state = [x0; xg];
guess.phase(1).control = [0; 0];

bounds.phase(2).initialtime.lower = ts_min;
bounds.phase(2).initialtime.upper = ts_max;
bounds.phase(2).initialstate.lower = xMin;
bounds.phase(2).initialstate.upper = xMax;
bounds.phase(2).finaltime.lower = ts_min+tm;
bounds.phase(2).finaltime.upper = ts_max+tm;
bounds.phase(2).state.lower = xMin;
bounds.phase(2).state.upper = xMax;
bounds.phase(2).control.lower = uMin;
bounds.phase(2).control.upper = uMax;
bounds.phase(2).finalstate.lower = xMin;
bounds.phase(2).finalstate.upper = xMax;
bounds.eventgroup(2).lower = 0;
bounds.eventgroup(2).upper = 0;

guess.phase(2).time = [(ts_min+ts_max)/2; (ts_min+ts_max)/2+tm];
guess.phase(2).state = [x0; xf];
guess.phase(2).control = [0; 0];

mesh.method = 'hp1';
mesh.tolerance = 0.01;
mesh.maxiteration = 8;
% mesh.colpointsmin = 4;
% mesh.colpointsmax = 10;

% mesh.phase(1).colpoints = 4*ones(1,10);
% mesh.phase(1).fraction = 0.1*ones(1,10);
% mesh.phase(2).colpoints = 4*ones(1,10);
% mesh.phase(2).fraction = 0.1*ones(1,10);

setup.name = 'Mobile-Sensor-Optimal-Trajectory-Multi';
setup.functions.continuous = @mobileSensorMultiContinuous;
setup.functions.endpoint = @mobileSensorMultiEndpoint;
setup.bounds = bounds;
setup.guess = guess;
setup.auxdata = auxdata;
setup.mesh = mesh;
setup.nlp.solver = 'snopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.method = 'RPMintegration';
setup.derivatives.dependencies = 'sparse';

output = gpops2(setup);

%%
save('result.mat', 'output');

%% plot

p1_x1_res = output.result.solution.phase(1).state(:,1);
p2_x1_res = output.result.solution.phase(2).state(:,1);

p1_x2_res = output.result.solution.phase(1).state(:,2);
p2_x2_res = output.result.solution.phase(2).state(:,2);

for k = 1:1:L;
    xt1_res(:,k) = [output.result.solution.phase(1).state(:,3+k*2-1);...
                    output.result.solution.phase(2).state(:,3+k*2-1)];

    xt2_res(:,k) = [output.result.solution.phase(1).state(:,3+k*2);...
                   output.result.solution.phase(2).state(:,3+k*2)];
end

p1_time = output.result.solution.phase(1).time;
p2_time = output.result.solution.phase(2).time;

a = -30*pi/180;
b = -120*pi/180;
Cb = zeros(100,2);
for i = 1:1:100;
    ang = a + (b-a)*i/100;
    Cb(i,1) = OC(1) + RC*cos(ang);
    Cb(i,2) = OC(2) + RC*sin(ang);
end

figure;
plot(p1_x1_res, p1_x2_res, 'b');hold on;
plot(p2_x1_res, p2_x2_res, 'r');
plot(xt1_res(:,1), xt2_res(:,1), 'k:');
plot(xt1_res(:,2), xt2_res(:,2), 'k--');
plot(xt1_res(:,3), xt2_res(:,3), 'k-.');
plot(Cb(:,1), Cb(:,2), 'm');
% axis([-20 80 -50 50]);
axis equal;

tf1 = p1_time(end)
tf2 = p2_time(end)
objective = output.result.objective
