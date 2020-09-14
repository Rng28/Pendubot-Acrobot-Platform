function tau = acrobotControlLaw_LQR(model, x)
%
% OUTPUTS:
%    tau = control tau, return empty if this control law is not applicable.


%%%%
q  = x(1:2,:);
qd = x(3:4,:);
%
% unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
q(1,:) = q(1,:) - 2*pi*floor(q(1,:)./(2*pi));
q(2,:) = q(2,:) - 2*pi*floor((q(2,:) + pi)./(2*pi));

%%%%
x_des = [pi-deg2rad(0); 0; 0; 0];

%%%% Linearization
persistent A_lin B_lin K S;
% 
if isempty(A_lin) || isempty(B_lin) || isempty(K) || isempty(S)
    
    [M,C,G,F,B,dGdq] = acrobotManipulatorEquation(model, x_des);
    
    A_lin = zeros(4,4);
    A_lin(1:2,1:2) = 0;
    A_lin(1:2,3:4) = eye(2);
    A_lin(3:4,1:2) = -M\dGdq;
    A_lin(3:4,3:4) = -M\C;
    
    B_lin = zeros(4,2);
    B_lin(3:4,:) = M\B;
    Q = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
    [K,S] = lqr(A_lin, B_lin, 20*Q, 1*eye(2));
end

x_tilda = [q;qd] - x_des; % x_current - x_desired

if  x_tilda'*S*x_tilda <40%1.5
    tau = K*(x_des - [q;qd]);
else
    tau = [];
end

end