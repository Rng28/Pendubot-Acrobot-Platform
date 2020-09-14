function [] = Plot_tau( t, tau,cu )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global mode;
    figure;
    subplot(3,1,1)
    plot (t,tau(1,:));
    xlabel('time');
    ylabel('torque(Nm)');
    title('output torque')
    
    if ~isempty(cu)
    subplot(3,1,2)
    
    plot (t,cu(1,:));
    xlabel('time');
    ylabel('torque(Nm)');
    title('desired torque')
    end
    
    if ~isempty(mode)
    subplot(3,1,3)
    plot (t,mode(1:length(t)));
    xlabel('time');
    title('mode')
    legend('1 : LQR')
    end
end

