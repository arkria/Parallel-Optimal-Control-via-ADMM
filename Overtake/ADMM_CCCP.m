function [ veh_cell, delta_out, time_CCCP] = ADMM_CCCP( veh_cell, Neigh_dist, rho, max_step, epsi_rel, epsi_abs, obstacle )
%ADMM_CCCP 此处显示有关此函数的摘要
%   此处显示详细说明
    n = size(veh_cell,2);
    Np = veh_cell{1,1}.Np;
    delta_out = zeros(n,1);

    for i = 1:n
        veh_cell{1,i}.x_plan = veh_cell{1,i}.x_pre;
        veh_cell{1,i}.y_plan = veh_cell{1,i}.y_pre;
        veh_cell{1,i}.theta_plan = veh_cell{1,i}.theta_pre;
        veh_cell{1,i}.x_plan_temp = veh_cell{1,i}.x_plan;
        veh_cell{1,i}.y_plan_temp = veh_cell{1,i}.y_plan;
        veh_cell{1,i}.theta_plan_temp = veh_cell{1,i}.theta_plan;
    end
    
    time_CCCP = 0;
    veh_couple_mat = ADMM_coupleCheck(veh_cell, Neigh_dist);
    
    [f1,g1,rel_mat1] = ADMM_transfer(veh_cell, veh_couple_mat, obstacle);
    [delta_temp1, MPC_time, empty_tag] = ADMM_MPC(f1, g1, rel_mat1);
%     disp(delta_temp1(1,:))
%     pause(1);
    
%     [f,g,rel_mat] = ADMM_transfer2(veh_cell, veh_couple_mat, obstacle);
%     [delta_temp] = ADMM_biple(f, g, rel_mat, rho, max_step, epsi_rel, epsi_abs, delta_temp1, empty_tag);
%     
    delta_apply = delta_temp1;
%     
%         time_CCCP = etime(tI2, tI1)+time_ADMM+time_CCCP;
    for i = 1:n
        X_next = veh_cell{1,i}.A_tilde*[veh_cell{1,i}.x_now;veh_cell{1,i}.y_now;veh_cell{1,i}.theta_now]+ ...
            veh_cell{1,i}.B_tilde*delta_apply(i,:)'+veh_cell{1,i}.W_tilde;
        veh_cell{1,i}.x_plan = X_next(1:3:3*Np-2);
        veh_cell{1,i}.y_plan = X_next(2:3:3*Np-1);
        veh_cell{1,i}.theta_plan = X_next(3:3:3*Np);
    end
    for i = 1:n
        veh_cell{1,i}.x_pre = veh_cell{1,i}.x_plan;
        veh_cell{1,i}.y_pre = veh_cell{1,i}.y_plan;
        veh_cell{1,i}.theta_pre = veh_cell{1,i}.theta_plan;
        veh_cell{1,i}.action_pre = [delta_apply(i,2:end),delta_apply(i,end)]';
        delta_out(i,:) = delta_apply(i,1);
        delta_out(i,:) = Other_boundFun(delta_out(i,:), [veh_cell{1,i}.delta_l,veh_cell{1,i}.delta_u]);
    end     
end

