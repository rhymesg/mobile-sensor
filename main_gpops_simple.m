% Single-target optimal-control setup; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

clear all;

t0 = 0;
tm = 10;
ts_min = 5;
ts_max = 15;
x0 = [30, 0, 50, -50, pi*3/4, 1, 0, 0, 0, 1, 0, 0, 0, 1];
xf = [0, 0, 20, 0, pi*3/4, 500, 0, 0, 0, 500, 0, 0, 0, 500];
xMin = [-100, -100, -100, -100, -pi, -1000*ones(1,9)];
xMax = -xMin;
uMin = -pi/5;
uMax = pi/5;

OC = [20 0]; % center of operational area
RC = 40; % radius of operational area
alt = 20; % altitude of UA
sig_R = [0.07 0.07 0.1]; % standard deviation of sensor noise
V = 3; % speed of UA

auxdata.OC = OC;
auxdata.RC = RC;
auxdata.tm = tm;
auxdata.alt = alt;
auxdata.sig_R = sig_R;
auxdata.V = V;

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
bounds.eventgroup(1).lower = zeros(1,16);
bounds.eventgroup(1).upper = zeros(1,16);

guess.phase(1).time = [t0; (ts_min+ts_max)/2];
xg = [x0(1:2) OC(1)+RC*cos(pi/4), OC(2)-RC*sin(pi/4), x0(5:end)];
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

setup.name = 'Mobile-Sensor-Optimal-Trajectory-Simple';
setup.functions.continuous = @mobileSensorSimpleContinuous;
setup.functions.endpoint = @mobileSensorSimpleEndpoint;
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

%% plot

p1_x1_res = output.result.solution.phase(1).state(:,3);
p2_x1_res = output.result.solution.phase(2).state(:,3);

p1_x2_res = output.result.solution.phase(1).state(:,4);
p2_x2_res = output.result.solution.phase(2).state(:,4);

p1_xt1_res = output.result.solution.phase(1).state(:,1);
p2_xt1_res = output.result.solution.phase(2).state(:,1);

p1_xt2_res = output.result.solution.phase(1).state(:,2);
p2_xt2_res = output.result.solution.phase(2).state(:,2);


p1_time = output.result.solution.phase(1).time;
p2_time = output.result.solution.phase(2).time;

for k = 1:1:101;
    Cb(k,1) = OC(1) + RC*cos(2*pi/100*(k-1));
    Cb(k,2) = OC(2) + RC*sin(2*pi/100*(k-1));
end

figure;
plot(p1_x1_res, p1_x2_res, 'b');hold on;
plot(p2_x1_res, p2_x2_res, 'r');
plot(p1_xt1_res, p1_xt2_res, 'k:');
plot(p2_xt1_res, p2_xt2_res, 'k--');
plot(Cb(:,1), Cb(:,2), 'm');
% axis([-20 80 -50 50]);

tf1 = p1_time(end)
tf2 = p2_time(end)
objective = output.result.objective
