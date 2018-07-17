function [ z_out, empty_tag ] = Centralized( f, g, rel_mat )
%CENTRALIZED 此处显示有关此函数的摘要
%   此处显示详细说明
    options = optimoptions('quadprog','Display','off');
    empty_tag = 0;
    [c_size, p_size] = size(rel_mat);
    Np = size(g{1,1}.z_temp,1);
    z_out = zeros(p_size, Np);
    H = zeros(p_size*Np);
    F = zeros(p_size*Np,1);
    A = [];
    B = [];
    for i = 1:c_size
        n = size(f{1,i}.rece_info,2);
        m = size(f{1,i}.A_o,1);
        A = [A;zeros(m, p_size*Np)];
        B = [B;zeros(m, 1)];
        for j = 1:n
            row = f{1,i}.rece_info(1,j);
            or = f{1,i}.rece_info(2,j);
            for k = 1:n
                col = f{1,i}.rece_info(1,k);
                oc = f{1,i}.rece_info(2,k);
                H((row-1)*Np+1:(row-1)*Np+Np, (col-1)*Np+1:(col-1)*Np+Np) = H((row-1)*Np+1:(row-1)*Np+Np, (col-1)*Np+1:(col-1)*Np+Np)+f{1,i}.H_o((or-1)*Np+1:(or-1)*Np+Np, (oc-1)*Np+1:(oc-1)*Np+Np);
            end
            F((row-1)*Np+1:(row-1)*Np+Np, 1) = F((row-1)*Np+1:(row-1)*Np+Np, 1)+f{1,i}.f_o((or-1)*Np+1:(or-1)*Np+Np, 1);
            if ~isempty(f{1,i}.A_o)
                A(end-m+1:end,(row-1)*Np+1:(row-1)*Np+Np) = f{1,i}.A_o(:,(or-1)*Np+1:(or-1)*Np+Np);
                B(end-m+1:end,1) = f{1,i}.b_o;
            end
            
        end
    end
    H = H+H';
    AN = quadprog(H,F,A,B,[],[],[],[],[],options);
    
    if isempty(AN)
        empty_tag = 1;
    end
    for i = 1:p_size
        z_out(i,:) = AN((i-1)*Np+1:(i-1)*Np+Np); 
    end
end

