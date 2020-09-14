function [x_dsr, xd_dsr, xdd_dsr] = desiredOperTrajectory(t)

    x_dsr   = [0.95*(1.5-cos(pi.*t)); 0.95*(1.5-sin(pi.*t))];
    xd_dsr  = [0.95*(pi*sin(pi*t)); -0.95*(pi*cos(pi*t))];
    xdd_dsr = [0.95*(pi^2*cos(pi*t)); 0.95*(pi^2*sin(pi*t))];

    % x_dsr   = [1*(4-cos(pi.*t)); 1*(0-sin(pi.*t))];
    % xd_dsr  = [1*(pi*sin(pi*t)); -1*(pi*cos(pi*t))];
    % xdd_dsr = [1*(pi^2*cos(pi*t)); 1*(pi^2*sin(pi*t))];

    % x_dsr   = [2.2214;0];
    % xd_dsr  = [0;0];
    % xdd_dsr = [0;0];
end
