function [ veh_struct ] = veh_posChange( veh_struct, x_distance, y_distance )
%VEH_POSCHANGE 此处显示有关此函数的摘要
%   此处显示详细说明
    % ====================== x =============================
    veh_struct.x_ini = veh_struct.x_ini+x_distance;
    veh_struct.x_period = veh_struct.x_period+x_distance;
    veh_struct.x_now = veh_struct.x_now+x_distance;
    veh_struct.x_tar = veh_struct.x_tar+x_distance;
    veh_struct.x_ref = veh_struct.x_ref+x_distance;
    veh_struct.x_pre = veh_struct.x_pre+x_distance;
    
    % ===================== y =============================
    veh_struct.y_ini = veh_struct.y_ini+y_distance;
    veh_struct.y_period = veh_struct.y_period+y_distance;
    veh_struct.y_now = veh_struct.y_now+y_distance;
    veh_struct.y_tar = veh_struct.y_tar+y_distance;
    veh_struct.y_ref = veh_struct.y_ref+y_distance;
    veh_struct.y_pre = veh_struct.y_pre+y_distance;
    
    % ==================== mat =============================
    veh_struct.H_tar = veh_struct.H_tar+[x_distance, y_distance, 0];
    veh_struct.real_path = veh_struct.real_path+[x_distance, y_distance, 0];
    


end

