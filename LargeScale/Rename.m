% clear
% clc
% close all
% %%
% v = 18; L = 3; R_veh = 1.8; deltaSet = linspace(-pi/6, pi/6, 100); 
% deltaT = 0.05; Np = 20; res = -2; qc = 0; qp = 10; R_safe = 0; Neigh_dist = 25; 
% alpha = 0.3; epsi_rel = 0.01; epsi_abs = 0.03; rho = 1000; max_step = 10000;
% tar_dist = 5; replan_dist = 50;
% road_width = 4;
% %% 创建障碍物，场景
% obstacle = [1, 2; 2, 2; 2, 1; 1, 1];
% Campus = Env_con(136, 136, road_width, obstacle, -2);
% %% 基础路径
% %% D
% % 半径为 7， 转角 0.4049
% lm_ODOR = [68; 0; -1000];
% um_ODOR = [Campus.width; 68-1; 1000];
% veh_ODOR = veh_con(74, 9.75, pi/2, 126, 62, 0, [74, 54.75, pi/2; 81, 62, 0; 126, 62, 0], [0; -0.4049; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ODOR, um_ODOR, qc, qp, 5);
% 
% % 半径为 9， 转角 0.3218
% lm_ODIR = [68; 0; -1000];
% um_ODIR = [Campus.width; 68-1; 1000];
% veh_ODIR = veh_con(74, 10, pi/2, 126, 66, 0, [74, 57, pi/2; 83, 66, 0; 126, 66, 0], [0; -0.3218; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ODIR, um_ODIR, qc, qp, 5);
% 
% % 直线
% lm_ODOU = [68+1; 0; -1000];
% um_ODOU = [76-1; Campus.height; 1000];
% veh_ODOU = veh_con(74, 10, pi/2, 74, 126, pi/2, [74, 126, pi/2], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ODOU, um_ODOU, qc, qp, 5);
% 
% % 半径为 14， 转角 0.2111
% lm_IDOL = [0; 0; -1000];
% um_IDOL = [76-1; 76-1; 1000];
% veh_IDOL = veh_con(70, 10, pi/2, 10, 74, pi, [70, 60, pi/2; 56, 74, pi; 10, 74, pi], [0; 0.2111; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IDOL, um_IDOL, qc, qp, 5);
% 
% % 半径为 10， 转角 0.2915
% lm_IDIL = [0; 0; -1000];
% um_IDIL = [76-1; 76-1; 1000];
% veh_IDIL = veh_con(70, 10, pi/2, 10, 70, pi, [70, 60, pi/2; 60, 70, pi; 10, 70, pi], [0; 0.2915; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IDIL, um_IDIL, qc, qp, 5);
% 
% % 直线
% lm_IDIU = [68+1; 0; -1000];
% um_IDIU = [76-1; Campus.height; 1000];
% veh_IDIU = veh_con(70, 10, pi/2, 70, 126, pi/2, [70, 126, pi/2], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IDIU, um_IDIU, qc, qp, 5);
% 
% %% L
% % 半径为 7， 转角 0.4049
% lm_OLOD = [0; 0; -1000];
% um_OLOD = [68-1; 68-1; 1000];
% veh_OLOD = veh_con(9.75, 62, 0, 62, 10, -pi/2, [54.75, 62, 0; 62, 55, -pi/2; 62, 10, -pi/2], [0; -0.4049; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OLOD, um_OLOD, qc, qp, 5);
% 
% % 半径为 9， 转角 0.3218
% lm_OLID = [0; 0; -1000];
% um_OLID = [68-1; 68-1; 1000];
% veh_OLID = veh_con(10, 62, 0, 66, 10, -pi/2, [57, 62, 0; 66, 53, -pi/2; 66, 10, 0], [0; -0.3218; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OLID, um_OLID, qc, qp, 5);
% 
% % 直线
% lm_OLOR = [0; 60+1; -1000];
% um_OLOR = [Campus.width; 68-1; 1000];
% veh_OLOR = veh_con(10, 62, 0, 126, 62, 0, [126, 62, 0], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OLOR, um_OLOR, qc, qp, 5);
% 
% % 半径为 14， 转角 0.2111
% lm_ILOU = [0; 60+1; -1000];
% um_ILOU = [76-1; Campus.height; 1000];
% veh_ILOU = veh_con(10, 66, 0, 74, 126, pi/2, [60, 66, 0; 74, 80, pi/2; 74, 126, pi], [0; 0.2111; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ILOU, um_ILOU, qc, qp, 5);
% 
% % 半径为 10， 转角 0.2915
% lm_ILIU = [0; 60+1; -1000];
% um_ILIU = [76-1; Campus.height; 1000];
% veh_ILIU = veh_con(10, 66, 0, 70, 126, pi/2, [60, 66, 0; 70, 76, pi/2; 70, 126, pi/2], [0; 0.2915; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ILIU, um_ILIU, qc, qp, 5);
% 
% % 直线
% lm_ILIR = [0; 60+1; -1000];
% um_ILIR = [Campus.width; 68-1; 1000];
% veh_ILIR = veh_con(10, 66, 0, 126, 66, 0, [126, 66, 0], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ILIR, um_ILIR, qc, qp, 5);
% 
% %% U
% % 半径为 7， 转角 0.4049
% lm_OUOL = [0; 68+1; -1000];
% um_OUOL = [68-1; Campus.height; 1000];
% veh_OUOL = veh_con(62, 126.25, -pi/2, 10, 74, pi, [62, 81.25, -pi/2; 55, 74, pi; 10, 74, 0], [0; -0.4049; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OUOL, um_OUOL, qc, qp, 5);
% 
% % 半径为 9， 转角 0.3218
% lm_OUIL = [0; 68+1; -1000];
% um_OUIL = [68-1; Campus.height; 1000];
% veh_OUIL = veh_con(62, 126, -pi/2, 10, 70, pi, [62, 79, -pi/2; 53, 70, pi; 10, 70, pi], [0; -0.3218; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OUIL, um_OUIL, qc, qp, 5);
% 
% % 直线
% lm_OUOD = [60+1; 0; -1000];
% um_OUOD = [68-1; Campus.height; 1000];
% veh_OUOD = veh_con(62, 126, -pi/2, 62, 10, -pi/2, [62, 10, -pi/2], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OUOD, um_OUOD, qc, qp, 5);
% 
% % 半径为 14， 转角 0.2111
% lm_IUOR = [60+1; 60+1; -1000];
% um_IUOR = [Campus.width; Campus.height; 1000];
% veh_IUOR = veh_con(66, 126, -pi/2, 126, 62, 0, [66, 76, -pi/2; 80, 62, 0; 126, 62, 0], [0; 0.2111; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IUOR, um_IUOR, qc, qp, 5);
% 
% % 半径为 10， 转角 0.2915
% lm_IUIR = [60+1; 60+1; -1000];
% um_IUIR = [Campus.width; Campus.height; 1000];
% veh_IUIR = veh_con(66, 126, -pi/2, 126, 66, 0, [66, 76, -pi/2; 76, 66, 0; 126, 66, 0], [0; 0.2915; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IUIR, um_IUIR, qc, qp, 5);
% 
% % 直线
% lm_IUID = [60+1; 0; -1000];
% um_IUID = [68-1; Campus.height; 1000];
% veh_IUID = veh_con(66, 126, -pi/2, 66, 10, -pi/2, [66, 10, -pi/2], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IUID, um_IUID, qc, qp, 5);
% 
% %% R
% % 半径为 7， 转角 0.4049
% lm_OROU = [68+1; 68+1; -1000];
% um_OROU = [Campus.width; Campus.height; 1000];
% veh_OROU = veh_con(126.25, 74, pi, 74, 126, pi/2, [81.25, 74, pi; 74, 81, pi/2; 74, 126, pi/2], [0; -0.4049; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OROU, um_OROU, qc, qp, 5);
% 
% % 半径为 9， 转角 0.3218
% lm_ORIU = [68+1; 68+1; -1000];
% um_ORIU = [Campus.width; Campus.height; 1000];
% veh_ORIU = veh_con(126, 74, pi, 70, 126, pi/2, [79, 74, pi; 70, 83, pi/2; 70, 126, pi/2], [0; -0.3218; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_ORIU, um_ORIU, qc, qp, 5);
% 
% % 直线
% lm_OROL = [0; 68+1; -1000];
% um_OROL = [Campus.width; 76-1; 1000];
% veh_OROL = veh_con(126, 74, pi, 10, 74, pi, [10, 74, pi], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_OROL, um_OROL, qc, qp, 5);
% 
% % 半径为 14， 转角 0.2111
% lm_IROD = [60+1; 0; -1000];
% um_IROD = [Campus.width; 76-1; 1000];
% veh_IROD = veh_con(126, 70, pi, 62, 10, -pi/2, [76, 70, pi; 62, 56, -pi/2; 62, 10, -pi/2], [0; 0.2111; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IROD, um_IROD, qc, qp, 5);
% 
% % 半径为 10， 转角 0.2915
% lm_IRID = [60+1; 0; -1000];
% um_IRID = [Campus.width; 76-1; 1000];
% veh_IRID = veh_con(126, 70, pi, 66, 10, -pi/2, [76, 70, pi; 66, 60, -pi/2; 66, 10, -pi/2], [0; 0.2915; 0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IRID, um_IRID, qc, qp, 5);
% 
% % 直线
% lm_IRIL = [0; 68+1; -1000];
% um_IRIL = [Campus.width; 76-1; 1000];
% veh_IRIL = veh_con(126, 70, pi, 10, 70, pi, [10, 70, pi], [0], 10, L, deltaT, R_veh, R_safe, deltaSet, res, Np, lm_IRIL, um_IRIL, qc, qp, 5);
% 
% %% 具体车辆建模
% % 只需改变veh.start_time and veh.v
% veh1 = veh_ODOR; veh2 = veh_ODIR; veh3 = veh_ODOU;
% veh4 = veh_IDOL; veh5 = veh_IDIL; veh6 = veh_IDIU;
% veh7 = veh_OLOD; veh8 = veh_OLID; veh9 = veh_OLOR;
% veh10 = veh_ILOU; veh11 = veh_ILIU; veh12 = veh_ILIR;
% veh13 = veh_OUOL; veh14 = veh_OUIL; veh15 = veh_OUOD;
% veh16 = veh_IUOR; veh17 = veh_IUIR; veh18 = veh_IUID;
% veh19 = veh_OROU; veh20 = veh_ORIU; veh21 = veh_OROL;
% veh22 = veh_IROD; veh23 = veh_IRID; veh24 = veh_IRIL;
% %% Astar生成参考轨迹
% n = 24; 
% veh_group = cell(1,n);
% real_path = cell(1,n);
% for i = 1:n
%     veh_group{1,i} = eval(['veh',num2str(i)]);
%     veh_group{1,i}.start_time = 5+(i-1)*20;
%     veh_group{1,i} = Path_generate(veh_group{1,i}, Campus); % Astar生成参考轨迹
%     real_path{1,i} = [veh_group{1,i}.x_now, veh_group{1,i}.y_now, veh_group{1,i}.theta_now]; % real_path 用途？？
% end
%%
clear
clc
close all
load('DATA.mat');
m = 0;

%%
% % 1 ODOU
% m = m+1; id = 3; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 1;Veh_group{1,m}.v = 14;Veh_group{1,m}.v_temp = 14;
% % 2 OROL
% m = m+1; id = 21; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 15;Veh_group{1,m}.v = 12;Veh_group{1,m}.v_temp = 12;
% % 3 IDIL
% m = m+1; id = 5; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 2;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 4 OUOD
% m = m+1; id = 15; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 3;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 5 ILIR
% m = m+1; id = 12; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 4;

% % 6 IUIR
% m = m+1; id = 17; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 50+1;Veh_group{1,m}.v = 11;Veh_group{1,m}.v_temp = 11;
% % 7 IRID
% m = m+1; id = 23; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 50+4;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 8 ODOU
% m = m+1; id = 3; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 50+13;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 12;
% % 9 OLID
% m = m+1; id = 8; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 50+2;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% 
% % 10 OUOD
% m = m+1; id = 15; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 100+6;Veh_group{1,m}.v = 11;Veh_group{1,m}.v_temp = 11;
% % 11 OROL
% m = m+1; id = 21; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 100+5;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 12 IDIU
% m = m+1; id = 6; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 100+15;Veh_group{1,m}.v = 13;Veh_group{1,m}.v_temp = 13;
% % 13 ILOU
% m = m+1; id = 10; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 100+1;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;

% 2 ODOU
m = m+1; id = 3; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 120+15;Veh_group{1,m}.v = 12;Veh_group{1,m}.v_temp = 12;
% 3 ILIU
m = m+1; id = 11; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 120+2;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% 4 OROL
m = m+1; id = 21; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 120+3;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% 5 IUID
m = m+1; id = 18; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 120+4;

% % 6 IRID
% m = m+1; id = 23; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 160+1;Veh_group{1,m}.v = 11;Veh_group{1,m}.v_temp = 11;
% % 7 IDIL
% m = m+1; id = 5; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 160+4;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 8 OLOR
% m = m+1; id = 9; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 160+13;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 12;
% % 9 OUIL
% m = m+1; id = 14; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 160+2;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;

% % 10 OROL
% m = m+1; id = 21; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 200+16;Veh_group{1,m}.v = 11;Veh_group{1,m}.v_temp = 14;
% % 11 IDIU
% m = m+1; id = 6; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 200+2;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;
% % 12 ILIR
% m = m+1; id = 12; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 200+1;Veh_group{1,m}.v = 12;Veh_group{1,m}.v_temp = 12;
% % 13 IUIR
% m = m+1; id = 17; Veh_group{1,m} = veh_group{1,id}; Veh_group{1,m}.start_time = 200+10;Veh_group{1,m}.v = 10;Veh_group{1,m}.v_temp = 10;


n = m;

%%
tic
time_flag = 0;
while true
    time_flag = time_flag + 1;
    disp(['time_flag = ', num2str(time_flag)]);
    
    Active_mat = [];
    Sleep_mat = [];
    for i = 1:n
        if time_flag >= Veh_group{1,i}.start_time;
            Active_mat = [Active_mat, i];
        else
            Sleep_mat = [Sleep_mat, i];
        end
    end
    for i = 1:n
        Veh_group{1,i} = Start_judge_CCA(Veh_group{1,i}, time_flag);
        Veh_group{1,i} = ADMM_stateUpdate(Veh_group{1,i}); 
    end 
    if ~isempty(Active_mat)
        i_temp = 1;
        for i = Active_mat
            Veh_temp{1,i_temp} = Veh_group{1,i};
            i_temp = i_temp+1;
        end
        [Veh_temp, delta_out] = ADMM_CCCP(Veh_temp, Neigh_dist, rho, max_step, epsi_rel, epsi_abs, obstacle);
        i_temp = 1;
        for i = Active_mat
            Veh_group{1,i} = Veh_temp{1,i_temp};
            i_temp = i_temp+1;
        end
    end
    
    i_temp = 1;
    for i = 1:n
        if any(Active_mat==i)
            Delta_out(i,1) = delta_out(i_temp);
            i_temp = i_temp+1;
        else
            Delta_out(i,1) = 0;
        end
    end
    % 状态更新，场景展示
    for i = 1:n
        Veh_group{1,i} = veh_struct_model(Veh_group{1,i}, Delta_out(i,1));
    end
    Env_con_CCA_show(Campus, Veh_group, 3.4/3.4, time_flag);
    % 终止判断
    terminal_tag = 0;
    for i = 1:n
        [Veh_group{1,i}, terminal_tag] = Termi_judge_CCA(Veh_group{1,i}, tar_dist, replan_dist, Np, terminal_tag);
    end
    disp(terminal_tag);
    if terminal_tag == n
        break
    end
    
end
toc
%%

