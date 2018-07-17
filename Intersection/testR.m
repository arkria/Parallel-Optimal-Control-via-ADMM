clear
clc
close all
%%
v = 18; L = 3; R_veh = 2; deltaSet = linspace(-pi/6, pi/6, 11); 
deltaT = 0.05; Np = 15; res = -2; qc = 0; qp = 10; R_safe = 1; Neigh_dist = 25; 
alpha = 0.3; epsi_rel = 0.01; epsi_abs = 0.03; rho = 1000; max_step = 10000;
tar_dist = 5; replan_dist = 50;
road_width = 4;
%% 创建障碍物，场景，车辆
obstacle = [1, 2; 2, 2; 2, 1; 1, 1];
Campus = Env_con(150, 30, road_width, obstacle, -2);
lm = [0; 0; -1000];
um = [Campus.height; Campus.height; 1000];
veh1 = veh_con(11, 140, -pi/2, 11, 10, -pi/2, [11, 10, -pi/2], 16, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm, um, qc, qp, 30);
% veh2 = veh_con(11, 140, -pi/2, 11, 10, -pi/2, [11, 10, -pi/2], 11, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm, um, qc, qp, 1);
%%
delta = veh1.deltaSet(2);
while true
    veh1 = veh_struct_model(veh1, delta);
    if abs(veh1.theta_now-pi/2)<=abs(v/L*tan(delta)*0.05);
        break
    end
end