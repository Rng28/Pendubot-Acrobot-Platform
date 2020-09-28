function [ tau,mode] = acrobotControlLaw_LQR(x,wdw,a,b,p,d)
%
% OUTPUTS:
%    tau : control torque, return empty if this control law is not applicable.
%    mode : LQR mode, ensure entering this part of code

mode = 1;
SMALL_NUM_VEC = deg2rad([0.02; 0.02]);

%%%%
q  = x(1:2,:);
qd = x(3:4,:);
%

% unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi] for top position
q(1,:) = q(1,:) - 2*pi*floor(q(1,:)./(2*pi));
q(2,:) = q(2,:) - 2*pi*floor((q(2,:) + pi)./(2*pi));

% unwrap angles q(1) to [-pi,pi] and q(2) to [0,2pi] for bottom position
%q(2,:) = q(2,:) - 2*pi*floor(q(2,:)./(2*pi));
%q(1,:) = q(1,:) - 2*pi*floor((q(1,:) + pi)./(2*pi));

%%%%
persistent K S;
x_des = [pi; 0; 0; 0];
no_case = 1; 
if no_case == 1
    x_des = [pi; 0; 0; 0]; %top

    K=[-24.963*a -23.823*b -4.6989*p -2.8317*d;
        0 0 0 0];

    S=[53.7693 46.9694 10.5400 3.9514;
        46.9694 42.8717 9.3543 3.5092;
        10.5400 9.3543 2.1515 0.8061;
        3.9514 3.5092 0.8061 0.3034];  

elseif no_case == 2
     x_des = [0; pi; 0; 0]; %bottom

     K=[11.6434*a 12.7794*b 0.0779*p 1.7291*d;
         0 0 0 0];

     S=[8.1022    8.2150   -0.4970    0.7777;
        8.2150   10.7193   -0.6431    0.9950;
       -0.4970   -0.6431    0.0627   -0.0904;
        0.7777    0.9950   -0.0904    0.1396];

elseif no_case == 3
    x_des = [pi; 0; 0; 0]; %top

    K=[-25.1045*a -23.9685*b -4.7430*p -2.8731*d;
        0 0 0 0];

    S=[54.6272 47.7579 10.7482 4.0819;
       47.7579 43.5969 9.5451 3.6272;
       10.7482 9.5451 2.2005 0.8352;
       4.0819 3.6272 0.8352 0.3184];
   
elseif no_case == 4
    x_des = [pi; 0; 0; 0]; %top

    K=[-25.3884*a -24.2524*b -4.8296*p -2.9550*d;
        0 0 0 0];

    S=[56.3272 49.3325 11.1626 4.3446;
        49.3325 45.0528 9.9269 3.8660;
         11.1626 9.9269 2.2983 0.8940;
         4.3446 3.8660 0.8940 0.3492];

elseif no_case == 5
    x_des = [pi; 0; 0; 0]; %top

    K=[-25.3957*a -24.2597*b -4.8319*p -2.9571*d;
        0 0 0 0];

    S=[56.3712 49.3733 11.1734 4.3515;
       49.3733 45.0905 9.9368 3.8722;
       11.1734 9.9368 2.3009 0.8955;
       4.3515 3.8722 0.8955 0.3501];  
end   


x_tilda = [q;qd] - x_des; % x_current - x_desired

if  x_tilda'*S*x_tilda < wdw
    xerror = x_des - [q;qd];
    tau = (K*xerror);
else
    tau = [];
end

end