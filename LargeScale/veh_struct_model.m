function [veh_struct] = veh_struct_model( veh_struct, delta_k )
    x_k = veh_struct.x_now; y_k = veh_struct.y_now; theta_k = veh_struct.theta_now;
    x_kp1 = x_k+veh_struct.deltaT*veh_struct.v*cos(theta_k);
    y_kp1 = y_k+veh_struct.deltaT*veh_struct.v*sin(theta_k);
    x_kp1 = roundn(x_kp1,veh_struct.res);
    y_kp1 = roundn(y_kp1,veh_struct.res);
    theta_kp1 = theta_k+veh_struct.deltaT*veh_struct.v/veh_struct.L*tan(delta_k);
    theta_kp1 = roundn(theta_kp1, -4); % 这里的精度必须是0.0001，否则无法达到终点
    if theta_kp1 > pi
        theta_kp1 = theta_kp1 - 2*pi;
    elseif theta_kp1 < -pi
        theta_kp1 = theta_kp1 + 2*pi;
    else
        theta_kp1 = theta_kp1;
    end
    veh_struct.x_now = x_kp1;
    veh_struct.y_now = y_kp1;
    veh_struct.theta_now = theta_kp1;
    veh_struct.real_path = [veh_struct.real_path; veh_struct.x_now, veh_struct.y_now, veh_struct.theta_now];
end

