function tau = acrobotControlLaw_RobustControl(model, x)

D  = [zeros(2); eye(2)];
Kp = diag([25 25]);
Kd = diag([10 10]);
K  = [Kp Kd];
H  = [zeros(2) eye(2); zeros(2) zeros(2)];
H_tilda = H - D*K;
P = 1*eye(4);
Q = lyap(H_tilda', P);

D = [0;0;1;1];

end
