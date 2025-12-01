function results = do_friction_model_fitting(fileName, arg1)

    data = load(fileName);
    
    vel = data(:,1);
    cur = data(:,2);
    
    step_num = length(vel)/4;
    
    if(arg1 == 1)

        for i = 1:step_num-1
            if(cur(i) >= cur(step_num*2 - i))
                refined_cur(i) = cur(step_num*2 - i);
                refined_vel(i) = vel(step_num*2 - i);
            else
                refined_cur(i) = cur(i);
                refined_vel(i) = vel(i);
            end
        end
        
        refined_cur(step_num) = 0;
        refined_vel(step_num) = 0;
        
        for i = 1:step_num-1
            if(cur(step_num*2 + i) <= cur(step_num*4 - i))
                refined_cur(step_num+i) = cur(step_num*4 - i);
                refined_vel(step_num+i) = vel(step_num*4 - i);
            else
                refined_cur(step_num+i) = cur(step_num*2 + i);
                refined_vel(step_num+i) = vel(step_num*2 + i);
            end
        end
        
        cf_cur = refined_cur(1:step_num);
        cf_vel = refined_vel(1:step_num);
        
        lsqr_vel = zeros(2, length(cf_vel));
        
        for i = 1:length(cf_cur)
            
            lsqr_vel(:, i) = [sign(cf_vel(i)); cf_vel(i)];
        
        end
        
        mu = cf_cur*pinv(lsqr_vel);

    end


    results = mu;


end

