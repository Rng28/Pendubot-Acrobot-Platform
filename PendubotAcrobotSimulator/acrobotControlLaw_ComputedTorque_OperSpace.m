function tau = acrobotControlLaw_ComputedTorque_OperSpace(t,model,x)

%%%%
q  = x(1:2,:);
qd = x(3:4,:);

%%%% Jacobian
J = acrobotJacobian(model, q);

%%%% Current state in cartesian space
[~, p] = acrobotFKine(model,q);
pd     = J*qd;

%%%% Desired trajectory
[p_dsr, pd_dsr, pdd_dsr] = desiredOperTrajectory(t);

Kp=diag([10 10]); Kd=diag([1 1]);
%V = Kp*(p_dsr - p) - Kd*(pd);
V = pdd_dsr + Kp*(p_dsr - p) + Kd*(pd_dsr - pd);


[M,C,G,F,B] = acrobotManipulatorEquation(model, x);

wm = model.wm; wc = model.wc; wg = model.wg; wf = model.wf;

tau = (M.*wm)*(pinv(J)*V)+(C.*wc)*qd+G.*wg+(F.*wf)*qd;

end