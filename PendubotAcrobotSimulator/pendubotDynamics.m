function xdot = pendubotDynamics(model, x, u)
%
% INPUTS:
%    model: struct
%    x: [4,1] = [q1; q2; q1d; q2d]
%    u: scalar = input torque
%
% OUTPUTS:
%    xdot: [4,1] = [q1d; q2d; q1dd; q2dd]

q  = x(1:2,:);
qd = x(3:4,:);

[M,C,G,F,B] = acrobotManipulatorEquation(model, x);

A_bar = M \ (-C*qd - G - F*qd);
B_bar = M \ B;

xdot = [qd; A_bar + B_bar*u];

end