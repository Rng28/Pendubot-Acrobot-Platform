function animate(model, tspan, t, x, x_dsr)

s = get(0, 'ScreenSize');
figure('Name', 'Acrobot Simulation',...
    'outerposition', [s(3)*0.1 s(4)*0.1 s(3)*0.8 s(4)*0.8]);
clf;

% Plot q1 & q2
%
subplot(3,1,3);
hold on;
plot(t, rad2deg(x(1,:)), '-r', 'LineWidth',1);
plot(t, rad2deg(x(2,:)), '-b', 'LineWidth',1);
xlabel('t');
ylabel('joint angle (degree)');
legend('q_1', 'q_2')

q1_ode_marker = plot(t(1), rad2deg(x(1,1)), 'ro');
q2_ode_marker = plot(t(1), rad2deg(x(2,1)), 'bo');
hold off;

subplot(3,1,[1 2]); hold on;
cla
title(sprintf('ODE45, t = %6.4f',0));
if ~isempty(x_dsr)
    plot(x_dsr(1,:), x_dsr(2,:), 'g')
end
drawAcrobot(0, x(:,1), model);
hold off;
pause(2);

% Animation
%
sim_time = 0;
tic; % Start stopwatch timer for simulation

while sim_time < tspan(end)
    
    %Interpolate to get the new point at current time:
    sim_time = toc;
    xq_ode = interp1(t', x', sim_time, 'spline')';
    
    %clf;
    subplot(3,1,[1 2]); hold on;
    %
    cla
    title(sprintf('ODE45, t = %6.4f',sim_time));
    if ~isempty(x_dsr)
        plot(x_dsr(1,:), x_dsr(2,:), 'g')
    end
    drawAcrobot(sim_time, xq_ode, model);

    hold off;
    
    % Update maker
    set(q1_ode_marker, 'xData', sim_time);
    set(q1_ode_marker, 'yData', rad2deg(xq_ode(1)));
    %
    set(q2_ode_marker, 'xData', sim_time);
    set(q2_ode_marker, 'yData', rad2deg(xq_ode(2)));
    
    drawnow;
    
    % Update simulation time
    sim_time = toc;
end

end