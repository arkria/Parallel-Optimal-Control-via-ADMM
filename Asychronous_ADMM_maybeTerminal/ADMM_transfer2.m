function [f, g, rel_mat] = ADMM_transfer2( veh_cell, couple_mat, obstacle )
%ADMM_TRANSFER 此处显示有关此函数的摘要
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
    
    if sum(sum(couple_mat))~= 0
        i = n;
        for ii = 1:n-1
            for jj = ii+1:n
                if couple_mat(ii,jj) == 1
                    i = i+1;
                    veh_s = veh_cell{1,ii};
                    veh_s_N = [veh_s.x_now;veh_s.y_now;veh_s.theta_now];
                    veh_r = veh_cell{1,jj};
                    veh_r_N = [veh_r.x_now;veh_r.y_now;veh_r.theta_now];
                    R_safe = veh_s.R_safe;
                    f{1,i}.H_o = 0*eye(2*veh_s.Np);
                    f{1,i}.f_o = zeros(2*veh_s.Np,1);
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
                    f{1,i}.A_o = -Psi*Gamma*B_check;
                    f{1,i}.A_o = roundn(f{1,i}.A_o, -8);
                    f{1,i}.b_o = -D_c+Psi*Gamma*[veh_s.A_tilde*veh_s_N+veh_s.W_tilde; veh_r.A_tilde*veh_r_N+veh_r.W_tilde];          
                    g1_l_s = veh_s.delta_l*ones(veh_s.Np,1); 
                    g1_l_r = veh_r.delta_l*ones(veh_r.Np,1); 
                    g1_u_s = veh_s.delta_u*ones(veh_s.Np,1);
                    g1_u_r = veh_r.delta_u*ones(veh_r.Np,1);
                    f{1,i}.lb_o = [g1_l_s;g1_l_r]; f{1,i}.ub_o = [g1_u_s;g1_u_r];
                end
            end
        end
        %% 定义与索引相关的信息及参数初始化
        rel_mat = zeros(m+n,n);
        for i = 1:n
            for j = 1:n
                if i == j
                    rel_mat(i,j) = 1; 
                end          
            end
        end

        k = n;
        for i = 1:n-1
            for j = i+1:n
                if couple_mat(i,j) == 1
                    k = k+1;
                    rel_mat(k,i) = 1;
                    rel_mat(k,j) = 1;
                end
            end
        end
        [c_size, p_size] = size(rel_mat);
        %% 定义master g
        pos_mat = zeros(c_size,p_size);
        for i = 1:c_size
            PP = 1;
            for j = 1:p_size
                if rel_mat(i,j) == 1
                    pos_mat(i,j) = PP;
                    PP = PP+1;
                end
            end
        end
        
        for i = 1:p_size
            g{1,i}.N = sum(rel_mat(:,i));
            g{1,i}.z_temp = veh_cell{1,i}.Delta_hat;
            g{1,i}.rece_info = zeros(2, g{1,i}.N);
            g{1,i}.rece_mat = zeros(Np, g{1,i}.N);
            pd = find(rel_mat(:,i) == 1);
            g{1,i}.rece_info(1,:) = pd;
            for j = 1:g{1,i}.N
                g{1,i}.rece_info(2,j) = pos_mat(pd(j),i);
            end
            g{1,i}.mat_size = Np;
        end
        %% 定义其他参数
        
        for i = 1:c_size
            if i <= p_size
                f{1,i}.lambda = zeros(1, Np);
                f{1,i}.rece_info = [i;1];
                f{1,i}.rece_mat = zeros(Np, 1);
                f{1,i}.mat_size = Np;
            else
                f{1,i}.lambda = zeros(1, 2*Np);
                f{1,i}.rece_info = zeros(2,2);
                pd = find(rel_mat(i,:)==1);
                f{1,i}.rece_info(1,:) = pd;
                f{1,i}.rece_info(2,:) = [1,2];
                f{1,i}.z_temp = [g{1,f{1,i}.rece_info(1,1)}.z_temp;g{1,f{1,i}.rece_info(1,2)}.z_temp];
                f{1,i}.rece_mat = zeros(2*Np, 1);
                f{1,i}.mat_size = 2*Np;
            end
        end      
    else
        g = [];
        rel_mat = [];
    end
    
    
end

