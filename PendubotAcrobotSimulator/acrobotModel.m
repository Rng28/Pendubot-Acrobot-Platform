function m = acrobotModel()

    model.l1  = 200/1000;
    model.l2  = 200/1000;
    model.lc1 = 148.09/1000; %-2.44 COM of link 1
    model.lc2 = 111.57/1000; % COM of link 2
    model.m1  = 79.74/1000;   %165.24g
    model.m2  = 21/1000;
    model.Ic1 = 407364.52/1000^3;   %590583.26
    model.Ic2 = 100178.38/1000^3;
    model.I1  = model.Ic1 + model.m1 * model.lc1^2;
    model.I2  = model.Ic2 + model.m2 * model.lc2^2;
    model.b1  = 0.0008; % dumper of joint 1
    model.b2  = 0.0004; % dumper of joint 2
    %
    model.B   = [1 0; 0 0];
    %
    % model disturbance
    %
    model.wm = 1;
    model.wc = 1;
    model.wg = 1;
    model.wf = 1;
    %
    % environment parameters
    model.g = 9.81;
m = model;

end