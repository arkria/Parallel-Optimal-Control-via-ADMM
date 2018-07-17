function action = veh_con_inver_model( state_1, state_2, veh_struct )
%VEH_CON_INVER_MODEL 此处显示有关此函数的摘要
%   此处显示详细说明
    up_thro = max(veh_struct.deltaSet);
    down_thro = min(veh_struct.deltaSet);
    theta_k = state_1(3);
    theta_kp1 = state_2(3);
    if (theta_kp1-theta_k)<veh_struct.deltaT*veh_struct.v/veh_struct.L*tan(down_thro)-0.1
        theta_kp1 = theta_kp1+2*pi;
    elseif (theta_kp1-theta_k)>veh_struct.deltaT*veh_struct.v/veh_struct.L*tan(up_thro)+0.1
        theta_kp1 = theta_kp1-2*pi;
    else
        theta_kp1 = theta_kp1;
    end
    action = atan((theta_kp1-theta_k)/(veh_struct.deltaT*veh_struct.v/veh_struct.L));
end

