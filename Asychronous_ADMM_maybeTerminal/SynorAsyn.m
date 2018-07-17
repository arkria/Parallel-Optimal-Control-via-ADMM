clear
clc
load('Compare_n324s127.mat')
Neigh_dist = 25;

index_vect = [10, 3, 6,7, 22, 2, 18, 8, 34, 1, 30, 9, 11, 15, 5, 19, 23, ...
    14, 17, 20, 35, 13, 29, 21, 12, 27, 4, 31, 24, 26, 16, 32, 36, 25, 28, 33];
index_vect = [index_vect, index_vect+36, index_vect+72,...
    index_vect+108, index_vect+144, index_vect+180,...
    index_vect+216, index_vect+252, index_vect+288];
cen_time = [];
den_time = [];
step_save = [];
net_save = []; 
end_num = 324;
epsi_rel = 0.001;
epsi_abs = 0.03;

% % synchronous : ADMM_DR_sy
% rho =700;
% max_step = 20000;

% asynchronous : ADMM_DR_asy_ARock (ADMM_DR_asy是没有采用ARock算法的版本)
rho = 100;
max_step = 100000; 

sy_save = [];
asy_save = [];
for veh_n = 5
    veh_cell = cell(1,veh_n);
    for i = 1:veh_n
        veh_cell{1,i} = Veh_cell{1,index_vect(i)};
    end

    veh_couple_mat = ADMM_coupleCheck(veh_cell, Neigh_dist);
    [f1,g1,rel_mat1] = ADMM_transfer(veh_cell, veh_couple_mat, obstacle);
    [delta_temp1, empty_tag, MPC_time] = ADMM_MPC(f1, g1, rel_mat1);

    [f,g,rel_mat] = ADMM_transfer2(veh_cell, veh_couple_mat, obstacle);
    tic
    [delta_temp_sy, ADMM_step_sy, test_save, rho_mat, A_set_save] = ADMM_DR_asy_ARock(f, g, rel_mat, rho, max_step, epsi_rel, epsi_abs, delta_temp1, empty_tag);
    toc
    disp(['============= vehicle number:', num2str(veh_n),' ============='])
    close all
    plot(log10(test_save(:,1)))
    hold on
    plot(log10(test_save(:,2)))
    plot(log10(test_save(:,3)))
    plot(log10(test_save(:,4)))
    legend('primal radius','dual radius','primal criterion','dual criterion')
end





