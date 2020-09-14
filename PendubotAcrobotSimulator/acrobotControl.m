function [u,cu, KE, PE] = acrobotControl(t,model,x,controlLaw, torqueLimit)

switch controlLaw
    case 'EnergyShaping'    
       [ tau, KE, PE] = acrobotControlLaw_EnergyShaping(model, x);
        
    case 'Collocated'
        tau = acrobotControlLaw_collocated_PFL(model,x);
        
    case 'NonCollocated'
        tau = acrobotControlLaw_noncollocated_PFL(model,x);
        
    case 'LQR'
        tau = acrobotControlLaw_LQR_Only(model,x);

    case 'ComputedTorque'
        tau = acrobotControlLaw_ComputedTorque(t,model,x);

    case 'ComputedTorque_PD'
        tau = acrobotControlLaw_ComputedTorque_PD_Only(t,model,x);
        
    case 'JacobianTranspose'
        tau = acrobotControlLaw_JacobianTranspose(t,model,x);
        
    case 'ComputedTorque_OperSpace'
        tau = acrobotControlLaw_ComputedTorque_OperSpace(t,model,x);
        
    otherwise
        tau = [0; 0];
        KE = 0;
        PE = 0;
end
cu = tau;
%%% Set torque limit
if torqueLimit > 0 
    u = sign(tau).*min(abs(tau), [torqueLimit; torqueLimit]);
else
    u = tau;
end

end