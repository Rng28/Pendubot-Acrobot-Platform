%function [K,S] = calcLQRgain()

%
% INPUTS:
%    model: struct
%    x: [4,1] = [q1; q2; q1d; q2d]
%
% OUTPUTS:
%    K: [4,4] = LQR gain
%    S: [4,4] = solution of the Riccati equation for the equivalent explicit state-space model


model = Model();

g  = model.g;
%
l1 = model.l1; lc1 = model.lc1; l2 = model.l2; lc2 = model.lc2;
m1 = model.m1; m2 = model.m2;
I1 = model.I1; I2 = model.I2;
b1 = model.b1; b2 = model.b2;

%%%%
q  = [pi;0];
qd = [0;0];
x = [q;qd];

%%%%
c = cos(q(1:2,:));  s = sin(q(1:2,:));  c12 = cos(q(1,:)+q(2,:));

[M, C, ~, F, ~, dGdq] = ManipulatorEquation(model, x);

%%%% linear state-space model xdot = Ax - Bu %%%
A3 = -inv(M) * dGdq;
A4 = -inv(M)*F;

A = [0 0 1 0;
     0 0 0 1;
     A3(1) A3(3) 0 0;
     A3(2) A3(4) 0 0];
 
B3 = inv(M) * [0;1];

B = [0 0 0 0;
     0 0 0 0;
     B3(1) 0 0 0
     B3(2) 0 0 0];

Q = eye(4);
R = eye(4);
[K,S] = lqr(A,B,Q,R)
[Kd,S] = lqrd(A,B,Q,R,0.001);