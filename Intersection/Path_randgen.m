function veh_struct = Path_randgen( veh_struct, Env_struct, time_length )
%PATH_RANDGEN 此处显示有关此函数的摘要
%   此处显示详细说明
    veh_struct.x_ini = 10+(Env_struct.width-20)*rand(1);
    veh_struct.y_ini = 10+(Env_struct.height-20)*rand(1);
    veh_struct.theta_ini = -pi+(2*pi)*rand(1);
    deltaSet = veh_struct.deltaSet;
    deltaSize = length(deltaSet);
%     x_temp = zeros(time_length, 1);
%     y_temp = zeros(time_length, 1);
%     theta_temp = zeros(time_length, 1);
%     action_temp = zeros(time_length, 1);
    
    x_temp(1) = veh_struct.x_ini;
    y_temp(1) = veh_struct.y_ini;
    theta_temp(1) = veh_struct.theta_ini;
    
    keep_time = 10;
    delta = 
    
    for t = 1:time_length-1
        randSeed = randperm(deltaSize);
        for k = 1:deltaSize
            delta = deltaSet(randSeed(1));
            [x_next, y_next, theta_next] = veh_con_model(x_temp(t), y_temp(t), theta_temp(t), veh_struct, delta);
            if Env_con_obscheck(Env_struct, [x_next, y_next]) == false
                action_temp(t) = delta;
                x_temp(t+1) = x_next;
                y_temp(t+1) = y_next;
                theta_temp(t+1) = theta_next;
                Env_con_reshow(Env_struct, [x_next, y_next], [1, 1], [1, 1])
                pause(0.1)
                break;
            else
                if k == deltaSize
                    break
                end
            end
        end
    end
    veh_struct.x_tar = x_temp(end);
    veh_struct.y_tar = y_temp(end);
    veh_struct.theta_tar = theta_temp(end);
    
    veh_struct.x_ref = x_temp';
    veh_struct.y_ref = y_temp';
    veh_struct.theta_ref = theta_temp';
    veh_struct.action_ref = action_temp';
    
    veh_struct.x_now = veh_struct.x_period; veh_struct.y_now = veh_struct.y_period; veh_struct.theta_now = veh_struct.theta_period;
    Np = veh_struct.Np;
    deltaX = veh_struct.x_ref(end)-veh_struct.x_ref(end-1);
    deltaY = veh_struct.y_ref(end)-veh_struct.y_ref(end-1);

    for i = 1:2*Np
        veh_struct.x_ref(end+1) = veh_struct.x_ref(end)+deltaX;
        veh_struct.y_ref(end+1) = veh_struct.y_ref(end)+deltaY;
        veh_struct.theta_ref(end+1) = veh_struct.theta_ref(end);
        veh_struct.action_ref(end+1) = 0;
    end
    veh_struct.x_pre = veh_struct.x_ref(1:Np);
    veh_struct.y_pre = veh_struct.y_ref(1:Np);
    veh_struct.theta_pre = veh_struct.theta_ref(1:Np);
    veh_struct.action_pre = veh_struct.action_ref(1:Np);

end

