function [tau, KE, PE] = acrobotControlLaw_EnergyShaping(model, x)
persistent tx;
global mode;
if isempty(tx)
    tx = 1;
else tx = tx+1;
end
tau = acrobotControlLaw_LQR(model, x);
KE = 0;
PE = 0;
if isempty(tau) % if LQR control law is not applicable, 
                %    then using Energy Shaping approach
    mode(tx) = 0;
    g  = model.g;
    %
    l1 = model.l1; lc1 = model.lc1; l2 = model.l2; lc2 = model.lc2;
    m1 = model.m1; m2 = model.m2;
    I1 = model.I1; I2 = model.I2;
    b1 = model.b1; b2 = model.b2;

    %%%%
    q  = x(1:2,:);
    qd = x(3:4,:);

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
    %
    ke = 1;%0.05;
    kd = 0;%ke/250;  %ke/15;
    ks = 0;%kd/80; %kd/10;
    q(1) = q(1) - 2*pi*floor(q(1)./(2*pi));
    d_tilda = pi-q(1);
    s_tilda = 0-qd(1);
    
    % P-saturation
    PD = d_tilda*kd+s_tilda*ks;
    if abs(PD) > 1
        PD = 1 * sign(PD);
    end
    
    tau = [(ke.*E_tilda.*qd(1)+PD); 0];   
else
    mode(tx) = 1;
end
    
end