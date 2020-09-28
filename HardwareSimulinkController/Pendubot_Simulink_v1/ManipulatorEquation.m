function [M, C, G, F, B, dGdq] = ManipulatorEquation(model, x)
%
% INPUTS:
%    model: struct
%    x: [4,1] = [q1; q2; q1d; q2d]
%
% OUTPUTS:
%    M: [2,2] = inertia matrix
%    C: [2,2] = coriolis and centrifugal terms
%    G: [2,1] = gravitational terms
%    F: [2,1] = Fiction force terms
%    dGdq: [2,2] = partial G / partial q

g  = model.g;
%
m1 = model.m1; m2 = model.m2;
l1 = model.l1; lc1 = model.lc1; l2 = model.l2; lc2 = model.lc2;
b1 = model.b1; b2 = model.b2;
I1 = model.I1;
I2 = model.I2;

q  = x(1:2,:);
qd = x(3:4,:);

c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));

m2l1lc2 = m2*l1*lc2;  % occurs often!

%%%% Find M(q), inertia matrix
M11 = I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2);
M12 = I2 + m2l1lc2*c(2);
M21 = M12;
M22 = I2;
%
M = [M11 M12; M21 M22];

%%%% Find C(q,qd), coriolis and centrifugal terms
C11 = -2*m2l1lc2*s(2)*qd(2);
C12 = -m2l1lc2*s(2)*qd(2);
C21 = m2l1lc2*s(2)*qd(1);
C22 = 0;
%
C = [ C11 C12; C21 C22 ];

%%%% Find G(q), gravitational terms
G1 = g*(m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12));
G2 = g*m2*lc2*s12;
G  = [ G1; G2 ];

%%%% Find F, Fiction force terms
F = [b1 0; 0 b2];

%%%%
% B = [0 0; 0 1];
%B = [1 0; 0 1];
B = model.B;

if nargout > 5
    %%%% Find dGdq, partial G / partial q
    dGdq11 = -g.*(m1.*lc1 + m2.*l1 + m2.*lc2);
    dGdq12 = -m2.*g.*lc2;
    dGdq21 = -m2.*g.*lc2;
    dGdq22 = -m2.*g.*lc2;
    %
    dGdq = [dGdq11 dGdq12; dGdq21 dGdq22];
end

end