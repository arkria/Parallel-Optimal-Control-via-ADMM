function [ f, g, CTP_mat ] = ADMM_transfer2( veh_cell, couple_mat, obstacle )
%ADMM_TRANSFER2 此处显示有关此函数的摘要
%   此处显示详细说明
    ob_dist = 2;
    n = size(veh_cell,2);
    m = 0.5*sum(sum(couple_mat));
    Np = veh_cell{1,1}.Np;
    
    for i = 1:n
        veh_tar = veh_cell{1,i};
        f{1,i}.H_o = veh_tar.Qc+veh_tar.B_tilde'*veh_tar.Qp*veh_tar.B_tilde;
        f{1,i}.H_o = 0.5*(f{1,i}.H_o+f{1,i}.H_o');
        f{1,i}.f_o = 2*(veh_tar.Gamma'*veh_tar.Qp*veh_tar.B_tilde)';        
        Q = [1 0 0; 0 1 0; 0 0 0];
        Phi = zeros(veh_tar.Np,3*veh_tar.Np);
        D_o = zeros(veh_tar.Np,1);
%         D_o = (veh_tar.R_veh+ob_dist)^2*ones(veh_tar.Np,1);
        for k = 1:veh_tar.Np
            ob_near = ob_map(veh_tar.x_plan(k),veh_tar.y_plan(k), obstacle);
            veh_P = [veh_tar.x_plan(k);veh_tar.y_plan(k);veh_tar.theta_plan(k)];
            ob_P = [ob_near.x;ob_near.y;0];
            for j = 1:veh_tar.Np
                if k == j                    
                    Phi(k, 3*(j-1)+1:3*j) = 2*(veh_P-ob_P)'*Q;
                end
            end
            D_o(k,1) = (veh_tar.R_veh+ob_dist)^2-ob_P'*Q*ob_P+veh_P'*Q*veh_P;
        end
        f{1,i}.A_o = -Phi*veh_tar.B_tilde;
        f{1,i}.A_o = roundn(f{1,i}.A_o, -8);
        f{1,i}.b_o = -D_o+Phi*veh_tar.A_tilde*[veh_tar.x_now;veh_tar.y_now;veh_tar.theta_now]+Phi*veh_tar.W_tilde;        
        g1_l = veh_tar.delta_l*ones(veh_tar.Np,1); g1_u = veh_tar.delta_u*ones(veh_tar.Np,1);
        f{1,i}.lb_o = g1_l; f{1,i}.ub_o = g1_u;
        f{1,i}.z_temp = veh_cell{1,i}.Delta_hat;
    end
    if sum(couple_mat) ~= 0
        CTP_mat = zeros(n,m);
        n_n = 1;
        i = 0;
        for ii = 1:n-1
            for jj = ii+1:n
                if couple_mat(ii,jj) == 1
                    CTP_mat(ii,n_n) = 1;
                    CTP_mat(jj,n_n) = 1;
                    n_n = n_n + 1;

                    i = i+1;
                    veh_s = veh_cell{1,ii};
                    veh_s_N = [veh_s.x_now;veh_s.y_now;veh_s.theta_now];
                    veh_r = veh_cell{1,jj};
                    veh_r_N = [veh_r.x_now;veh_r.y_now;veh_r.theta_now];
                    R_safe = veh_s.R_safe;
                    g{1,i}.H_o = 0*eye(2*veh_s.Np);
                    g{1,i}.f_o = zeros(2*veh_s.Np,1);
                    D_c = zeros(veh_s.Np,1);
                    Q = [1 0 0 -1 0 0; 0 1 0 0 -1 0];
                    Q_check = Q'*Q;
                    B_check = [veh_s.B_tilde, zeros(size(veh_s.B_tilde,1),size(veh_s.B_tilde,2));zeros(size(veh_s.B_tilde,1),size(veh_s.B_tilde,2)), veh_r.B_tilde];
                    Psi = zeros(veh_r.Np,6*veh_r.Np);
                    Gamma = zeros(6*veh_r.Np);
                    for k = 1:veh_r.Np
                        veh_CP = [veh_s.x_plan(k);veh_s.y_plan(k);veh_s.theta_plan(k);veh_r.x_plan(k);veh_r.y_plan(k);veh_r.theta_plan(k)];                   
                        for j = 1:veh_r.Np
                            if k == j
                                Psi(k, 6*(j-1)+1:6*j) = 2*veh_CP'*Q_check;
                            end
                        end
                        Gamma(6*(k-1)+1:6*(k-1)+3, 3*(k-1)+1:3*k) = eye(3);
                        Gamma(6*(k-1)+4:6*(k-1)+6, 3*veh_s.Np+3*(k-1)+1:3*veh_s.Np+3*k) = eye(3);
                        D_c(k,1) = (veh_s.R_veh+veh_r.R_veh+R_safe)^2+veh_CP'*Q_check*veh_CP;
                    end
                    g{1,i}.A_o = -Psi*Gamma*B_check;
                    g{1,i}.A_o = roundn(g{1,i}.A_o, -8);
                    g{1,i}.b_o = -D_c+Psi*Gamma*[veh_s.A_tilde*veh_s_N+veh_s.W_tilde; veh_r.A_tilde*veh_r_N+veh_r.W_tilde];          
                    g1_l_s = veh_s.delta_l*ones(veh_s.Np,1); 
                    g1_l_r = veh_r.delta_l*ones(veh_r.Np,1); 
                    g1_u_s = veh_s.delta_u*ones(veh_s.Np,1);
                    g1_u_r = veh_r.delta_u*ones(veh_r.Np,1);
                    g{1,i}.lb_o = [g1_l_s;g1_l_r]; g{1,i}.ub_o = [g1_u_s;g1_u_r];
                end
            end
        end

        %% 定义与索引相关的信息以及参数初始化
        for i = 1:n
            f{1,i}.per_set = [];
            for j = 1:m
                if CTP_mat(i,j) == 1
                    f{1,i}.per_set = [f{1,i}.per_set,j];  
                end
            end
            f{1,i}.per_size = size(f{1,i}.per_set,2);
            f{1,i}.lambda = zeros(f{1,i}.per_size, Np);
        end


        for j = 1:m
            g{1,j}.cen_set = [];
            for i = 1:n
                if CTP_mat(i,j) == 1
                    g{1,j}.cen_set = [g{1,j}.cen_set, i];
                end            
            end
            g{1,j}.z_temp = [f{1,g{1,j}.cen_set(1)}.z_temp;f{1,g{1,j}.cen_set(2)}.z_temp];
        end
    else
        g = [];
        CTP_mat = [];
    end
   
end

