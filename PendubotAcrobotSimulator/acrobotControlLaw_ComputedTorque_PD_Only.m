function tau = acrobotControlLaw_ComputedTorque_PD_Only(t,model,x)


%%%%
q  = x(1:2,:);
qd = x(3:4,:);
%
% unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
% q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
% q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));

%%%% Set point
% q_dsr   = [pi; 0];
% qd_dsr  = [0; 0];
% qdd_dsr = [0; 0];

%%%% Desired trajectory
[q_dsr, qd_dsr, qdd_dsr] = desiredJointTrajectory(t);

Kp=20*eye(2); Kd=20*eye(2);
V = Kp*(q_dsr - q) + Kd*(qd_dsr - qd);

[M,C,G,F,B] = acrobotManipulatorEquation(model, x);

wm = model.wm; wc = model.wc; wg = model.wg; wf = model.wf;

tau = qdd_dsr + V + G.*wg;

end