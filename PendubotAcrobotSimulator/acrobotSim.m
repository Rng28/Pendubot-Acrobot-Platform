function [t, x] = acrobotSim(model, x0, controlLaw, torqueLimit, tspan)

clear global; close all;

%%%% Acrobot model & enviroment parameters
if ~exist('model','var')
    model = acrobotModel();
end
 
%%%% Initial state:
if ~exist('x0','var')
    q1  = deg2rad(10);
    q2  = deg2rad(10);
    q1d = deg2rad(0);
    q2d = deg2rad(0);
    x0  = [q1;q2;q1d;q2d];  % Pack up initial state
end

%%%%
if ~exist('controlLaw','var')
    controlLaw = '';
end

space = 'JointSpace';

switch controlLaw
    case 'EnergyShaping'    
        model.B = [1 0; 0 0];
    case 'Collocated'
        model.B = [0 0; 0 1];
    case 'NonCollocated'
        model.B = [0 0; 0 1];
    case 'LQR'
        model.B = [1 0; 0 0];
    case 'ComputedTorque'
        model.B = [1 0; 0 1]; % fully actuated
    case 'ComputedTorque_PD'
        model.B = [1 0; 0 1]; % fully actuated
    case 'JacobianTranspose'
        model.B = [1 0; 0 1]; % fully actuated
        space   = 'OperationalSpace';
    case 'ComputedTorque_OperSpace'
        model.B = [1 0; 0 1]; % fully actuated
        space   = 'OperationalSpace';
    otherwise
        model.B = [0 0; 0 1];
end

%%%%
if ~exist('torqueLimit','var')
    torqueLimit = 0;
end

%%%%
if ~exist('tspan','var')
    tspan = [0,10];  % Time span for the simulation
end

%%%% Control law
%u = @(t,x)(0); % Motor torque (set to zero for passive dynamics)
u = @(t,x)( acrobotControl(t,model,x,controlLaw, torqueLimit) );

%%%% Run simulation with ODE45:
% options = odeset('RelTol',1e-8, 'AbsTol',1e-8);
% sol = ode45(@(t,x)(acrobotDynamics(model,x,u(t,x))), tspan, x0, options);
% %
% t = linspace(tspan(1),tspan(2),(tspan(2)-tspan(1))*100);
% x = deval(sol,t);

%%%% Run simulation with Euler's method:
dt = 0.001;
%
[t, x] = eulerMethod(...
    @(t,x)(acrobotDynamics(model,x,u(t,x))), tspan, x0, dt);
%%%% Run animation
if strcmp(space, 'JointSpace') 
    animate(model, tspan, t, x, []);
else
    x_dsr = desiredOperTrajectory(t);
    animate(model, tspan, t, x, x_dsr);
end

%%%% Show plot
%showPlot(t, x);

for i = 1:length(t)
[tau(:,i),cu(:,i), KE(i), PE(i)] = feval(u, t(i), x(:,i));
end
Plot_tau(t,tau ,cu);
Plot_energy(KE,PE,t);
end