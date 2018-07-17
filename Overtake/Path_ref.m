function veh_struct = Path_ref( veh_struct, Env_struct )
%PATH_REF 根据单个veh_struct 以及场景Env_struct生成车辆的参考轨迹
%  Iput: veh_struct, Env_struct 
%  Output: veh_struct (path_ref, path_pre)
%     H_tar = Dijistra_contain(veh_struct, Env_struct);
    H_tar = veh_struct.H_tar;
    for i = 1:size(H_tar,1)
        [partial_path,partial_action] = Astar_new(veh_struct, Env_struct, H_tar(i,1), H_tar(i,2), H_tar(i,3));
        veh_struct.x_now = partial_path(end,1); veh_struct.y_now = partial_path(end,2); veh_struct.theta_now = partial_path(end,3);
        veh_struct.x_ref = [veh_struct.x_ref;partial_path(:,1)];
        veh_struct.y_ref = [veh_struct.y_ref;partial_path(:,2)];
        veh_struct.theta_ref = [veh_struct.theta_ref;partial_path(:,3)];
        veh_struct.action_ref = [veh_struct.action_ref;partial_action(:)];
    end
  
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

