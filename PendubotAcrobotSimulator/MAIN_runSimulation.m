clear all; clear global;% close all;

%%%% Passive dynamics
model = acrobotModel();
model.b1  = 0.001;% dumper of joint 1
model.b2  = 0.0005;% dumper of joint 2
x0 = [deg2rad(-0); deg2rad(0.0); deg2rad(-0.01); deg2rad(0.0)]; % [q1; q2; q1d; q2d]
tspan = [0 10];
controlLaw = 'EnergyShaping';

torqueLimit = 0.5;

%%%% LQR
% model = acrobotModel();
% model.b1  = 0.0008; % dumper of joint 1
% model.b2  = 0.0004; % dumper of joint 2
% x0 = [deg2rad(180); deg2rad(0); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 10];
% controlLaw = 'LQR';
% torqueLimit = 20;


%%%% EnergyShaping
% global mode;
% model = acrobotModel();
% model.b1  = 0.0008; % dumper of joint 1
% model.b2  = 0.0004; % dumper of joint 2
% x0 = [deg2rad(20); deg2rad(0); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 10];
% controlLaw = 'EnergyShaping';
% torqueLimit = 0.1;

%%%% Collocated PFL
% model = acrobotModel();
% x0 = [deg2rad(10); deg2rad(0); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 20];
% controlLaw = 'Collocated';
% torqueLimit = 40;

% %%%%
[t,x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan);
%animate(model, tspan, t, x, []);

%% ComputedTorque
% model = acrobotModel();
% model.wm = 1;
% model.wc = 2;
% model.wg = 1;
% x0 = [deg2rad(5); deg2rad(5); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 10];
% controlLaw = 'ComputedTorque';
% torqueLimit = 0;
% 
% %%%%
% [t,x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan);
% %animate(model, tspan, t, x, []);
% 
% [q, qd, qdd] = desiredJointTrajectory(t);
% showPlot2(t,x, [q;qd]);

%%
% %%%% PD
% model = acrobotModel();
% model.wg = 0;
% x0 = [deg2rad(5); deg2rad(5); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 10];
% controlLaw = 'ComputedTorque_PD';
% torqueLimit = 0;
% 
% %%%%
% [t,x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan);
% %animate(model, tspan, t, x, []);
% 
% [q, qd, qdd] = desiredJointTrajectory(t);
% showPlot2(t,x, [q;qd]);

%%
% %%%% JacobianTranspose
% model = acrobotModel();
% model.wg = 1;
% x0 = [deg2rad(20); deg2rad(90); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 10];
% controlLaw = 'JacobianTranspose';
% torqueLimit = 0;
% 
% %%%%
% [t,x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan);
% %animate(model, tspan, t, x, []);
% 
% [p, pd, pdd] = desiredOperTrajectory(t);
% showPlot3(t, model, x, [p; pd]);
% %saveAnimation(model, tspan, t, x, [p; pd]);

%%
% %%%% ComputedTorque_OperSpace
% model = acrobotModel();
% model.wg = 1;
% x0 = [deg2rad(20); deg2rad(90); deg2rad(0); deg2rad(0)]; % [q1; q2; q1d; q2d]
% tspan = [0 20];
% controlLaw = 'ComputedTorque_OperSpace';
% torqueLimit = 0;
% 
% %%%%
% [t,x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan);
% %animate(model, tspan, t, x, []);
% 
% % [p, pd, pdd] = desiredOperTrajectory(t);
% % showPlot3(t, model, x, p);
% 