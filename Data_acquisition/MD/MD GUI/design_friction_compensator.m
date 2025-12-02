function results = design_friction_compensator(filesave_name, filename, e, t_Fs, t_Fc, Vmax, mu_V_desired, fit_model, gain)
    
    % User Input)
    % e:    Width of deadzone                   (unit: rad)
    % Vmax: Actuator maximum velocity           ( = Motor max velocity / gear ratio)
    % mu_V: Desired Linear Friction Coefficient (unit: A/rad/s)
    t_data = load(filename);

    velocity = t_data(:,2);  % output velocity (rad/s) (output)
    current =  t_data(:,3);   % input current (A)

    % model type
    Coulomb_Viscous = 1;
    Stribeck1       = 2;
    Stribeck2       = 3;

    % STEP 0) Set Weight
    weight = zeros(1, length(velocity));
    for i = 1:length(velocity)

        if (abs(velocity(i)) < 0.5)
            weight(i) = 1;
        else 
            weight(i) = 1;
        end
    end

    % STEP 1) Do Curve Fitting 
    actual_friction_model  = zeros(1, length(current));
    desired_friction_model = zeros(1, length(current));
    friction_comp_input    = zeros(1, length(current));
    LUT_SIZE      = 8193;
    resize_factor = (2*Vmax)/(LUT_SIZE-1);
  
    design_save_file = fopen(filesave_name, 'w');

    if (fit_model == Coulomb_Viscous)
    
        Initial = [0.5, 0.5];
        
        str = sprintf('Fc*tanh((pi/2)*x/%f) + b*x', e);
        ft = fittype(str);
        F=fitoptions('method','NonlinearLeastSquares','MaxIter',2000,'MaxFunEvals',600,'StartPoint',Initial);
        fm = fit(velocity, current, ft,F);

        Fc   = fm.Fc;
        b   = fm.b;    
    
        % STEP 2) Generate Compensator Input
        for i = 1:LUT_SIZE
            x = (i - ((LUT_SIZE-1)/2+1)) * resize_factor;
            actual_friction_model(i)  = Fc*tanh((pi/2)*x/e) + b*x;
            desired_friction_model(i) = mu_V_desired * x;
            friction_comp_input(i)    = gain * (actual_friction_model(i) - desired_friction_model(i));

            fprintf(design_save_file, '%.3f %.3f %.3f %.3f \n', x, actual_friction_model(i), desired_friction_model(i), friction_comp_input(i));
        end

    elseif (fit_model == Stribeck1)

        Initial = [0.5, 0.5];

        %weight = 1./(100*abs(velocity)+0.1);
                
        str = sprintf('(%f-(%f-%f)*exp(-(x/vs)^2))*tanh((pi/2)*x/%f) + b*x', t_Fs, t_Fs, t_Fc, e);
        ft = fittype(str);
        F=fitoptions('method','NonlinearLeastSquares','MaxIter',2000,'MaxFunEvals',600,'StartPoint',Initial, 'weight', weight);
        fm = fit(velocity, current, ft,F);

        vs   = fm.vs;
        b   = fm.b;    
    
        % STEP 2) Generate Compensator Input
        for i = 1:LUT_SIZE
            x = (i - ((LUT_SIZE-1)/2+1)) * resize_factor;
            actual_friction_model(i)  = tanh((pi/2)*x/e) * (t_Fs-(t_Fs-t_Fc)*exp(-(x/vs)^2)) + b*x;
            desired_friction_model(i) = mu_V_desired * x;
            friction_comp_input(i)    = gain * (actual_friction_model(i) - desired_friction_model(i));
            
            fprintf(design_save_file, '%.3f %.3f %.3f %.3f \n', x, actual_friction_model(i), desired_friction_model(i), friction_comp_input(i));
        end


    elseif (fit_model == Stribeck2)

        %Initial = [0.3, 0.65, 0.05, 0.37, -0.1];
        %weight = 1./(100*abs(velocity)+0.1);
        Initial = [0.5, 0.5, 0.5];

        str = sprintf('(%f-(%f-%f)*exp(-(x/vs)^2))*tanh((pi/2)*x/%f) + b*x + c*x^2*sign(x)', t_Fs, t_Fs, t_Fc, e);
        ft = fittype(str);
       % F=fitoptions('method','NonlinearLeastSquares','MaxIter',4000,'MaxFunEvals',1000,'StartPoint',Initial, 'weight', weight);
       
        F=fitoptions('method','NonlinearLeastSquares','MaxIter',4000,'MaxFunEvals',1000,'StartPoint',Initial, 'weight', weight);
        F.Lower = [0.01, 0, -1];
        F.Upper = [0.3, 1, 1];

        fm = fit(velocity, current, ft,F);

        vs   = fm.vs;
        b   = fm.b;
        c   = fm.c;       
 
        % STEP 2) Generate Compensator Input
        for i = 1:LUT_SIZE
            x = (i - ((LUT_SIZE-1)/2+1)) * resize_factor;
            actual_friction_model(i)  = tanh((pi/2)*x/e) * (t_Fs-(t_Fs - t_Fc)*exp(-(x/vs)^2)) + b*x + c*x^2*sign(x);
            desired_friction_model(i) = mu_V_desired * x;
            friction_comp_input(i)    = gain * (actual_friction_model(i) - desired_friction_model(i));

            fprintf(design_save_file, '%.3f %.3f %.3f %.3f \n', x, actual_friction_model(i), desired_friction_model(i), friction_comp_input(i));
        end

    end

    fclose(design_save_file);

    % STEP 3) Generate Look-Up Table (LUT)
    
    FC_LUT        = zeros(LUT_SIZE, 1);
    t_actual_friction_model = 0;

    for i = 1:LUT_SIZE
        t_vel = (i - ((LUT_SIZE-1)/2+1)) * resize_factor;
        
        if (fit_model == Coulomb_Viscous)
            t_actual_friction_model  = Fc*tanh((pi/2)*t_vel/e) + b*t_vel;

        elseif (fit_model == Stribeck1)
            t_actual_friction_model  = tanh((pi/2)*t_vel/e) * (t_Fs-(t_Fs-t_Fc)*exp(-(t_vel/vs)^2)) + b*t_vel;

        elseif (fit_model == Stribeck2)
            t_actual_friction_model  = tanh((pi/2)*t_vel/e) * (t_Fs-(t_Fs-t_Fc)*exp(-(t_vel/vs)^2)) + b*t_vel + c*t_vel^2*sign(t_vel);

        end

        t_desired_friction_model = mu_V_desired * t_vel;
        FC_LUT(i) = gain*(t_actual_friction_model- t_desired_friction_model);
    end
    
%     LUT_name = sprintf('FC_LUT_mdl%d.txt', fit_model);
%     writematrix(round(FC_LUT,8), LUT_name);
% 
    param1 = 0; param2 = 0; param3 = 0; param4 = 0; param5 = 0; param6 = gain;
    if (fit_model == Coulomb_Viscous)
        param1 = Fc;
        param2 = b;
    elseif (fit_model == Stribeck1)
        param1 = t_Fs;
        param2 = t_Fc;
        param3 = vs;
        param4 = b;
    elseif (fit_model == Stribeck2)
        param1 = t_Fs;
        param2 = t_Fc;
        param3 = vs;
        param4 = b;
        param5 = c;
    end
    results = [fit_model, param1, param2, param3, param4, param5, param6];
end

