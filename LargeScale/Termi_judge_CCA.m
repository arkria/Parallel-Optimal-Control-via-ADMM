function [veh_struct, terminal_tag] = Termi_judge_CCA( veh_struct, tar_dist, replan_dist, Np, terminal_tag )
%TERMI_JUDGE_CCA 此处显示有关此函数的摘要
%   此处显示详细说明
%     terminal_tag = 0;
    [compare_dist, ref_tag] = Other_refPathDist(veh_struct.x_now, veh_struct.y_now, veh_struct.x_ref, veh_struct.y_ref);
    if pdist([veh_struct.x_now, veh_struct.y_now;veh_struct.x_tar, veh_struct.y_tar]) <= tar_dist
        terminal_tag = terminal_tag+1;
        veh_struct.terminal = 1;
        veh_struct.x_pre = veh_struct.x_now*ones(Np,1); 
        veh_struct.y_pre = veh_struct.y_now*ones(Np,1);
        veh_struct.theta_pre = veh_struct.theta_now*ones(Np,1);
        veh_struct.v = 0;
    elseif compare_dist > replan_dist
        veh_struct.x_ref(ref_tag+1:end) = []; 
        veh_struct.y_ref(ref_tag+1:end) = [];
        veh_struct.theta_ref(ref_tag+1:end) = [];
        veh_struct.x_period = veh_struct.x_now;
        veh_struct.y_period = veh_struct.y_now;
        veh_struct.theta_period = veh_struct.theta_now;
        veh_struct = Path_ref(veh_struct, Campus);
    end

end

