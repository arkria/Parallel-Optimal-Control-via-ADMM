clear
clc
close all
%%
v = 18; L = 3; R_veh = 2; deltaSet = linspace(-pi/6, pi/6, 11); 
deltaT = 0.05; Np = 20; res = -2; qc = 0; qp = 10; R_safe = 0; Neigh_dist = 25; 
alpha = 0.3; epsi_rel = 0.01; epsi_abs = 0.03; rho = 1000; max_step = 10000;
tar_dist = 5; replan_dist = 50;
road_width = 4;
%% 创建障碍物，场景，车辆
obstacle = [1, 2; 2, 2; 2, 1; 1, 1];
Campus = Env_con(50, 70, road_width, obstacle, -2);
lm1 = [0; 0; -1000];
um1 = [38-1.2; 38-1.2; 1000];
lm2 = [30+1.2; 0; -1000];
um2 = [Campus.width; 38-1.2; 1000];
lm3 = [0; 30+1.2; -1000];
um3 = [Campus.width; 38-1.2; 1000];
veh1 = veh_con(36, 10, pi/2, 10, 36, pi, [36, 26.7682, pi/2; 26.7682, 36, pi; 10, 36, pi], [0; 0.3142; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm1, um1, qc, qp, 1);
% veh1 = veh_con(36, 10, pi/2, 10, 36, pi, [36, 21.8834, pi/2; 21.8834, 36, pi; 10, 36, pi], [0; 0.2094; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm1, um1, qc, qp, 1);
veh2 = veh_con(58, 36, pi, 32, 15, -pi/2, [41.2318, 36, pi; 32, 26.7682, -pi/2; 32, 15, -pi/2], [0; 0.3142; 0], 8, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm2, um2, qc, qp, 1);
% veh2 = veh_con(58, 36, pi, 32, 10, -pi/2, [46.1166, 36, pi; 32, 21.8834, -pi/2; 32, 10, -pi/2], [0; 0.2094; 0], 16, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm2, um2, qc, qp, 1);
veh3 = veh_con(10, 32, 0, 58, 32, 0, [58, 32, 0], [0], 13, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm3, um3, qc, qp, 10);

%% Astar生成参考轨迹
n = 3; 
veh_group = cell(1,n);
real_path = cell(1,n);
for i = 1:n
    veh_group{1,i} = eval(['veh',num2str(i)]);
    veh_group{1,i} = Path_generate(veh_group{1,i}, Campus); % Astar生成参考轨迹
    real_path{1,i} = [veh_group{1,i}.x_now, veh_group{1,i}.y_now, veh_group{1,i}.theta_now]; % real_path 用途？？
end
%%
tic
time_flag = 0;
while true
    time_flag = time_flag + 1;
    disp(['time_flag = ', num2str(time_flag)]);
    
    for i = 1:n
        veh_group{1,i} = Start_judge_CCA(veh_group{1,i}, time_flag);
        veh_group{1,i} = ADMM_stateUpdate(veh_group{1,i}); 
    end 
    [veh_group, delta_out] = ADMM_CCCP(veh_group, Neigh_dist, rho, max_step, epsi_rel, epsi_abs, obstacle);
    % 状态更新，场景展示
    for i = 1:n
        veh_group{1,i} = veh_struct_model(veh_group{1,i}, delta_out(i,1));
    end
    Env_con_CCA_show(Campus, veh_group, 3.4/3.4);
    % 终止判断
    terminal_tag = 0;
    for i = 1:n
        [veh_group{1,i}, terminal_tag] = Termi_judge_CCA(veh_group{1,i}, tar_dist, replan_dist, Np, terminal_tag);
    end
    disp(terminal_tag);
    if terminal_tag == n
        break
    end
    
end
toc
%%

