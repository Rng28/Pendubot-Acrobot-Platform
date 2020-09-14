function [] = Plot_energy( KE, PE,t )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    figure;
    plot (t,KE);
    hold on;
    plot (t,PE);
    plot (t,PE+KE);
    hold off;
    xlabel('time');
    ylabel('Energy');
    legend('KE','PE','total energy');

end

