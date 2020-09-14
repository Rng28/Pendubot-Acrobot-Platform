function [q, qd, qdd] = desiredJointTrajectory(t)

q   = [-(pi/2)*sin(t); -(pi/2)*sin(t)];
qd  = [-(pi/2)*cos(t); -(pi/2)*cos(t)];
qdd = [(pi/2)*sin(t); (pi/2)*sin(t)];

end