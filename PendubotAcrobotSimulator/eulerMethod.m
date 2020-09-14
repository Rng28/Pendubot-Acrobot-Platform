function [t, y] = eulerMethod(dfunc, tspan, x0, dt)
global mode;
t = tspan(1): dt: tspan(2);
y = zeros(size(x0,1), size(t,2));
y(:,1) = x0;
if isempty(mode)
mode(1:length(t)) = 0;
end
for i = 1:length(t)-1
    y(:,i+1) = y(:,i) + (feval(dfunc, t(i), y(:,i)))*dt;
end

end