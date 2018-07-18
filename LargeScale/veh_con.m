function veh_struct = veh_con( x_ini, y_ini, theta_ini, x_tar, y_tar, theta_tar, H_tar, A_tar, v, L, deltaT, R_veh , R_safe, deltaSet,res, Np, lm, um, qc, qp, start_time)
% theta为弧度制，范围为-pi~pi
%deltaSet为弧度制，取值范围为-pi/6~pi/6，这里将其等分为10份，其size为10*1
    %% 以下为用于Astar部分的数据
    % 出发时间
    veh_struct.start_time = start_time;
    % 初始位置
    veh_struct.x_ini = x_ini;
    veh_struct.y_ini = y_ini;
    veh_struct.theta_ini = theta_ini;
    % 
    veh_struct.x_period = x_ini;
    veh_struct.y_period = y_ini;
    veh_struct.theta_period = theta_ini;
    % 
    veh_struct.x_now = x_ini;
    veh_struct.y_now = y_ini;
    veh_struct.theta_now = theta_ini;
    % 目标位置
    veh_struct.x_tar = x_tar;
    veh_struct.y_tar = y_tar;
    veh_struct.theta_tar = theta_tar;
    veh_struct.H_tar = H_tar;
    veh_struct.A_tar = A_tar;
    % 记录路径
    veh_struct.real_path = [veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now];
    % 车辆参数
    veh_struct.v = v;
    veh_struct.v_temp = v;
    veh_struct.L = L;
    veh_struct.deltaSet = deltaSet; % 转角动作集合
    % 时空参数
    veh_struct.deltaT = deltaT; % 时间间隔
    veh_struct.R_veh = R_veh; % 车辆安全距离
    veh_struct.R_safe = R_safe; % 障碍物安全距离
    veh_struct.res = res; % 地图精度
        
    % 以下为用于MPC部分的数据
    veh_struct.Np = Np;
    veh_struct.delta_plan = [];
    veh_struct.x_plan = [];
    veh_struct.y_plan = [];
    veh_struct.theta_plan = [];
    veh_struct.x_plan_temp = [];
    veh_struct.y_plan_temp = [];
    veh_struct.theta_plan_temp = [];
    % 参考
    veh_struct.x_ref = [];
    veh_struct.y_ref = [];
    veh_struct.theta_ref = [];
    veh_struct.action_ref = [];
    % 预测
    veh_struct.x_pre = [];
    veh_struct.y_pre = [];
    veh_struct.theta_pre = [];
    veh_struct.action_pre = [];
    
    veh_struct.A_linear = [];
    veh_struct.B_linear = [];
    veh_struct.W_linear = [];
    veh_struct.A_tilde = [];
    veh_struct.B_tilde = [];
    veh_struct.W_tilde = [];
    veh_struct.Gamma = [];
    veh_struct.Delta_hat = [];
    veh_struct.LM = repmat(lm, Np, 1);
    veh_struct.UM = repmat(um, Np, 1);  
    
    for i = 1:Np
        tempqp(3*(i-1)+1:3*i) = [1 1 1]; 
    end
    veh_struct.Qc = qc*eye(veh_struct.Np);
    veh_struct.Qp = qp*diag(tempqp);
    
    % ADMM
    veh_struct.delta_l = min(veh_struct.deltaSet);
    veh_struct.delta_u = max(veh_struct.deltaSet);
        
    %以下为用于协调部分
    veh_struct.terminal = 0;

end

