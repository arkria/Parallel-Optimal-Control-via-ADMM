function [ z_out, empty_tag, MPC_time ] = ADMM_MPC( f, g, rel_mat, Act_mat, Sle_mat)
%ADMM_MPC 此处显示有关此函数的摘要
%   此处显示详细说明
    
    options = optimoptions('quadprog','Display','off');
    n = size(g,2);
    m = size(f,2) - n;
    Np = size(g{1,1}.z,1);
    H = zeros(n*Np, n*Np);
    F = zeros(n*Np, 1);
    A = zeros(n*9*Np+m*2*Np, n*Np);
    B = zeros(n*9*Np+m*2*Np, 1);
    empty_tag = 0;
    for i = 1:n
        H((i-1)*Np+1:i*Np, (i-1)*Np+1:i*Np) = f{1,i}.H_o;
        F((i-1)*Np+1:i*Np, 1) = f{1,i}.f_o;
        A((i-1)*9*Np+1:i*9*Np, (i-1)*Np+1:i*Np) = f{1,i}.A_o;
        B((i-1)*9*Np+1:i*9*Np, 1) = f{1,i}.b_o;
        Lb((i-1)*Np+1:i*Np, 1) = f{1,i}.lb_o;
        Ub((i-1)*Np+1:i*Np, 1) = f{1,i}.ub_o;
    end
    for i = n+1:n+m
        k = 1;
        for j = 1:n
            if rel_mat(i,j) == 1
                Temp = f{1,i}.A_o;
                A((i-1)*Np+1:i*Np,(j-1)*Np+1:j*Np) = Temp(:,k:k+Np-1);
                k = k+Np;
            end
        end
        B((i-1)*Np+1:i*Np,1) = f{1,i}.b_o;
    end
    H = H+H';
    
%     model = [];
%     model.Q = sparse(0.5*H);
%     model.A = sparse(A);
%     model.obj = F';
%     model.rhs = B';
%     model.lb = Lb';
%     model.ub = Ub';
%     model.sense = '<';
%     gurobi_write(model, 'qp.lp');
%     params.outputflag = 0;
%     t1 = clock;
%     results = gurobi(model, params);
%     t2 = clock;
%     if results.status(1) == 'I'
%         model = [];
%         model.Q = sparse(0.5*H);
%         model.A = sparse(A);
%         model.obj = F';
%         model.rhs = B';
%         model.sense = '<';
%         gurobi_write(model, 'qp.lp');
%         params.outputflag = 0;
%         t1 = clock;
%         results = gurobi(model, params);
%         t2 = clock;
%     end
%     Z = results.x;
%     Z = Other_boundFun(Z, [Lb,Ub]);
    
    t1 = clock;
    Z = quadprog(H, F, A, B, [], [], Lb, Ub, [], options);
    if isempty(Z)
        Z = quadprog(H, F, A, B, [], [], [], [], [], options);
        Z = Other_boundFun(Z, [Lb,Ub]);
        empty_tag = 1;
    end
    t2 = clock;
    MPC_time = etime(t2,t1);
    for i = 1:n
        z_out(i,:) = Z((i-1)*Np+1:i*Np);
    end
    
    
end

