function [ f, g ] = GenerateTopology( rel_mat, Np )
%GENERATETOPOLOGY 此处显示有关此函数的摘要
%   此处显示详细说明
    [c_size, p_size] = size(rel_mat);
    for i = 1:c_size
        n = sum(rel_mat(i,:));
        H_X = diag(rand(n*Np,1));
        H_U = orth(rand(n*Np,n*Np));
        f{1,i}.H_o = H_U'*H_X*H_U;
        f{1,i}.H_o = 0.5*(f{1,i}.H_o+f{1,i}.H_o');
        f{1,i}.f_o = f{1,i}.H_o*2*(rand(n*Np,1)-0.5);
        A_X = diag(rand(n*Np,1));
        A_U = orth(rand(n*Np,n*Np));
        f{1,i}.A_o = A_X'*A_U*A_X;
        f{1,i}.b_o = f{1,i}.A_o*2*(rand(n*Np,1)-0.5);
        f{1,i}.A_o = [];
        f{1,i}.b_o = [];
        f{1,i}.lb_o = [];
        f{1,i}.ub_o = [];
        f{1,i}.lambda = zeros(1,n*Np);
%         f{1.i}.rece_info = zeros(2, n);
        k = 0;
        for j = 1:p_size
            if rel_mat(i,j) == 1
                k = k+1;
                f{1,i}.rece_info(:,k) = [j;k];
            end
        end
        f{1,i}.rece_mat = zeros(n*Np,1);
        f{1,i}.mat_size = n*Np;
        f{1,i}.z_temp = zeros(n*Np,1);
    end
    
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
        g{1,i}.z_temp = zeros(Np, 1);
        g{1,i}.rece_info = zeros(2, g{1,i}.N);
        g{1,i}.rece_mat = zeros(Np, g{1,i}.N);
        pd = find(rel_mat(:,i) == 1);
        g{1,i}.rece_info(1,:) = pd;
        for j = 1:g{1,i}.N
            g{1,i}.rece_info(2,j) = pos_mat(pd(j),i);
        end
        g{1,i}.mat_size = Np;
    end
end

