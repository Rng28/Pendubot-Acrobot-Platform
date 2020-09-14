function tau = acrobotControlLaw_collocated_PFL(model,x)

tau = acrobotControlLaw_LQR(model, x);

if isempty(tau) % if LQR control law is not applicable, 
                %    then using Energy Shaping approach
    %%%%
    q  = x(1:2,:);
    qd = x(3:4,:);
    %
    % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
    q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
    q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));
    
    [M,C,G,F,B] = acrobotManipulatorEquation(model, x);
    
    %%%% Non-collocated PFL
    M21bar = M(2,1) - M(2,2)*pinv(M(1,2))*M(1,1);
    Phi = C*qd + G + B*qd;
    h2bar = Phi(2) - M(2,2)*pinv(M(1,2))*Phi(1);
    
    q1des = pi;
    
    kp=1; kd=10;
    v1 = kp*(q1des - q(1,:)) - kd*qd(1,:);
    
    tau = [0; M21bar*v1 + h2bar];
end

end