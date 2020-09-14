function tau = acrobotControlLaw_EnergyShaping_plus_PFL(model, x)

tau = acrobotControlLaw_LQR(model, x);

if isempty(tau) % if LQR control law is not applicable, 
                %    then using Energy Shaping approach

    g  = model.g;
    %
    l1 = model.l1; lc1 = model.lc1; l2 = model.l2; lc2 = model.lc2;
    m1 = model.m1; m2 = model.m2;
    I1 = model.I1; I2 = model.I2;
    b1 = model.b1; b2 = model.b2;

    %%%%
    q  = x(1:2,:);
    qd = x(3:4,:);
    %
    % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
    q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
    q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));

    %%%%
    c = cos(q(1:2,:));  s = sin(q(1:2,:));  c12 = cos(q(1,:)+q(2,:));
    
    [M,C,G,F,B] = acrobotManipulatorEquation(model, x);
    
    %%%% energy shaping
    E_des = m1.*g.* lc1 + m2.*g.*(l1+lc2);
    %
    KE = 0.5*qd.'*M*qd;
    PE = -m1*g*lc1*c(1) - m2*g*(l1*c(1)+lc2*c12);
    E = KE + PE;
    %
    E_tilda = E_des - E;
    
    %%%% Collocated PFL
    M22bar = M(2,2) - M(2,1)/M(1,1)*M(1,2);
    Phi = C*qd + G + B*qd;
    h2bar = Phi(2) - M(2,1)/M(1,1)*Phi(1);

    %%%%
    k1 = 1;
    k2 = 1;
    k3 = 10;
    %
    tau = M22bar*(-k1*q(2) - k2*qd(2)) + h2bar + k3.*E_tilda*qd(2);    
    
end
    
end