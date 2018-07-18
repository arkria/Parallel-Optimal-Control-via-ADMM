function veh_struct = ADMM_stateUpdateBackup( veh_struct )
%ADMM_STATEUPDATE 此处显示有关此函数的摘要
%   tag == 0, 表示还没有进行规划，此时按照参考轨迹来更新状态； tag == 1，表示进行了规划，有预测量，此时按照预测量来更新状态。
    if veh_struct.terminal == 1
        veh_struct.v = 0;
    elseif veh_struct.start == 0
        veh_struct.v = 0;
    end
    [~, flag] = Other_refPathDist(veh_struct.x_now, veh_struct.y_now, veh_struct.x_ref, veh_struct.y_ref);
    Np = veh_struct.Np;
    veh_struct.A_linear = [1 0 -veh_struct.v*veh_struct.deltaT*sin(veh_struct.theta_now); 0 1 veh_struct.v*veh_struct.deltaT*cos(veh_struct.theta_now); 0 0 1];
    veh_struct.B_linear = [-veh_struct.v^2*sin(veh_struct.theta_now)*veh_struct.deltaT^2/(2*veh_struct.L);veh_struct.v^2*cos(veh_struct.theta_now)*veh_struct.deltaT^2/(2*veh_struct.L);veh_struct.v*veh_struct.deltaT/veh_struct.L];
    veh_struct.W_linear = [veh_struct.v*(cos(veh_struct.theta_now)+veh_struct.theta_now*sin(veh_struct.theta_now))*veh_struct.deltaT;...
        veh_struct.v*(sin(veh_struct.theta_now)-veh_struct.theta_now*cos(veh_struct.theta_now))*veh_struct.deltaT; 0];
    A = veh_struct.A_linear; B = veh_struct.B_linear; W = veh_struct.W_linear;
    for i = 1:Np
        veh_struct.A_tilde(3*(i-1)+1:3*i,:) = A^i;
        sumW = 0;
        for j = 1:i
            sumW = sumW+A^(j-1)*W;
        end
        veh_struct.W_tilde(3*(i-1)+1:3*i,:) = sumW;
        for j = 1:Np
            if j <= i
                veh_struct.B_tilde(3*(i-1)+1:3*i,j) = A^(i-j)*B;
            else
                veh_struct.B_tilde(3*(i-1)+1:3*i,j) = [0;0;0];
            end             
        end
    end
    veh_struct.A_tilde = roundn(veh_struct.A_tilde,-8);

    if veh_struct.terminal == 0
        for i = 1:veh_struct.Np
            X_hat(3*(i-1)+1:3*i,:) = [veh_struct.x_ref(flag+i-1);veh_struct.y_ref(flag+i-1);veh_struct.theta_ref(flag+i-1)];
            Delta_hat(i,1) = veh_struct.action_pre(i);
        end
    else
        for i = 1:veh_struct.Np
            X_hat(3*(i-1)+1:3*i,:) = [veh_struct.x_now;veh_struct.y_now;veh_struct.theta_now];
            Delta_hat(i,1) = 0;
        end
    end
    
    veh_struct.Gamma = veh_struct.A_tilde*[veh_struct.x_now;veh_struct.y_now;veh_struct.theta_now]+veh_struct.W_tilde-X_hat;
    for i = 3:3:3*Np
        if veh_struct.Gamma(i)> pi
            veh_struct.Gamma(i) = 2*pi-veh_struct.Gamma(i);
        elseif veh_struct.Gamma(i)<-pi
            veh_struct.Gamma(i) = -2*pi-veh_struct.Gamma(i);
        end
    end
    veh_struct.Delta_hat = Delta_hat;
end

