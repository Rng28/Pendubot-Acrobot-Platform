function [p1,p2] = acrobotFKine(model,x)

q1 = x(1,:);
q2 = x(2,:);

l1 = model.l1;
l2 = model.l2;

x1 = l1*cos(q1-pi/2);
y1 = l1*sin(q1-pi/2);
x2 = x1 + l2*cos(q1+q2-pi/2);
y2 = y1 + l2*sin(q1+q2-pi/2);

p1 = [x1; y1];
p2 = [x2; y2];

end
