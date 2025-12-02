function results = do_sys_id_verification(t_J, t_B, t_fileID)

    data = load(t_fileID);
    
    cnt            = data(:,1);
    id_input       = data(:,2);
    friction_input = data(:,3);
    output_act     = data(:,4);

    s = tf('s');
    Gs = 1/(t_J*s + t_B);
    Gz =c2d(Gs, 0.001, 'zoh');
    Gz = minreal(Gz);
    [num, den] = tfdata(Gz, 'v');

    output_est = zeros(length(cnt), 1);

    fileID = fopen('sys_id_verification_with_sim.txt', 'w');

    for i = 1:length(id_input)

        if (i == 1) 
            output_est(i) = output_act(i);  
        else
            output_est(i) = -den(2)*output_est(i-1) + num(2)*id_input(i);  
        end

        fprintf(fileID, '%d %.3f %.3f %.3f %.3f \n', cnt(i), id_input(i), friction_input(i), output_act(i), output_est(i));
    end

   fclose(fileID); 

   results = 1;

   %results = [cnt, id_input, friction_input, output_act, output_est];
end