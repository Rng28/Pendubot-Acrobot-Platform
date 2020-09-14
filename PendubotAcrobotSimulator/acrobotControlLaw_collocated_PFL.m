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
    
    %%%% Collocated PFL
    M22bar = M(2,2) - M(2,1)/M(1,1)*M(1,2);
    Phi = C*qd + G + B*qd;
    h2bar = Phi(2) - M(2,1)/M(1,1)*Phi(1);
    
    q2des = atan(qd(1,:));
    
    kp=10; kd=1;
    v2 = kp*(q2des - q(2,:)) - kd*qd(2,:);
    
    tau = [0; M22bar*v2 + h2bar];
end

end