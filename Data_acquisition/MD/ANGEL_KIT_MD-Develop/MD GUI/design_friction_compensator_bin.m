function results = design_friction_compensator(filename, e, Vmax, mu_V_desired)
    
    % User Input)
    % e:    Width of deadzone                   (unit: rad)
    % Vmax: Actuator maximum velocity           ( = Motor max velocity / gear ratio)
    % mu_V: Desired Linear Friction Coefficient (unit: A/rad/s)
    t_data = load(filename);

    velocity = t_data(:,2);  % output velocity (rad/s) (output)
    current =  t_data(:,3);   % input current (A)

    % STEP 1) Do Curve Fitting 
    % (with odd 5th order polynomial, for f(x) = -f(-x);

    %Initial = [0.5, 0.5, 0.5, 0.5];

    Initial = [0.5, 0.5, 0.5];
    %str = sprintf('(mu_C)*tanh((pi/2)*x/(%f/2)) + (a*x + b*x^3 + c*x^5)', e);
    str = sprintf('(mu_C)*tanh((pi/2)*x/(%f/2)) + (a*x + b*x^3)', e);
    ft = fittype(str);
    F=fitoptions('method','NonlinearLeastSquares','MaxIter',2000,'MaxFunEvals',600,'StartPoint',Initial);
    fm = fit(velocity, current, ft,F);

    a    = fm.a;
    b    = fm.b;
    %c    = fm.c;
    c = 0;
    mu_C = fm.mu_C; 
    
%     actual_friction_model  = zeros(1, length(current));
%     desired_friction_model = zeros(1, length(current));
%     friction_comp_input    = zeros(1, length(current));
    
    % STEP 2) Generate Compensator Input
%     for i = 1:length(velocity)
%         
%         actual_friction_model(i)  = tanh((pi/2)*velocity(i)/e) * (mu_C + a*velocity(i) + b*velocity(i)^3 + c*velocity(i)^5);
%         desired_friction_model(i) = mu_V_desired * velocity(i);
%         friction_comp_input(i)    = actual_friction_model(i) - desired_friction_model(i);
%     end

    % STEP 3) Generate Look-Up Table (LUT)
    LUT_SIZE      = 8193;
    FC_LUT        = zeros(LUT_SIZE, 1);
    resize_factor = (2*Vmax)/(LUT_SIZE-1);

    for i = 1:LUT_SIZE
        t_vel = (i - ((LUT_SIZE-1)/2+1)) * resize_factor;
        %t_actual_friction_model  = mu_C*tanh((pi/2)*t_vel/(e/2)) + (a*t_vel + b*t_vel^3 + c*t_vel^5);
        t_actual_friction_model  = mu_C*tanh((pi/2)*t_vel/(e/2)) + (a*t_vel + b*t_vel^3);
        t_desired_friction_model = mu_V_desired * t_vel;
        FC_LUT(i) = t_actual_friction_model- t_desired_friction_model;
    end
    
    writematrix(round(FC_LUT,8), 'FC_LUT.txt');

    %results = [mu_C, a, b, c];
    results = [mu_C, a, b, c];
end

