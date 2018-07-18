clear
clc
close all
%%
v = 18; L = 3; R_veh = 2; deltaSet = linspace(-pi/6, pi/6, 11); 
deltaT = 0.05; Np = 15; res = -2; qc = 0; qp = 10; R_safe = 1; Neigh_dist = 25; 
alpha = 0.3; epsi_rel = 0.01; epsi_abs = 0.03; rho = 1000; max_step = 10000;
tar_dist = 5; replan_dist = 50;
road_width = 4;
%% �����ϰ������������
obstacle = [1, 2; 2, 2; 2, 1; 1, 1];
Campus = Env_con(50, 50, road_width, obstacle, -2);
lm = [0; 0; -1000];
um = [Campus.height; Campus.height; 1000];
R = 9;
delta = -0.3218;
veh1 = veh_con(10, 10, pi/2, 10+R, 10+R, 0, [10+R, 10+R, 0], [delta],9, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm, um, qc, qp, 30);
% veh2 = veh_con(11, 140, -pi/2, 11, 10, -pi/2, [11, 10, -pi/2], 11, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm, um, qc, qp, 1);
%%
% delta = veh1.deltaSet(2);
% while true
%     veh1 = veh_struct_model(veh1, delta);
%     if abs(veh1.theta_now-pi/2)<=abs(v/L*tan(delta)*0.05);
%         break
%     end3
% end
n = 1; 
veh_group = cell(1,n);
real_path = cell(1,n);
for i = 1:n
    veh_group{1,i} = eval(['veh',num2str(i)]);
    veh_group{1,i} = Path_generate(veh_group{1,i}, Campus); % Astar���ɲο��켣
    real_path{1,i} = [veh_group{1,i}.x_now, veh_group{1,i}.y_now, veh_group{1,i}.theta_now]; % real_path ��;����
end