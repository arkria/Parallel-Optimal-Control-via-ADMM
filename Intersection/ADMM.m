function [z_out,time_ADMM,tag,test_save,rho_mat] = ADMM( f, g, rel_mat, rho, max_step, epsi_pri, epsi_dual )
%ADMM 此处显示有关此函数的摘要
%   此处显示详细说明
%     time_ADMM = 0;
    
    options = optimoptions('quadprog','Display','off');
    [b_size, d_size] = size(rel_mat);
%     pri_cri = b_size;
%     dual_cri = d_size;
    z_out = zeros(d_size,size(g{1,1}.z,1));
    tag = 0;
    test_save = [];
    ADMM_tag = 0;
    rho_mat = [rho,0];
    
    %% 初始化 u
    parfor i = 1:b_size
        f{1,i}.u = zeros(size(f{1,i}.f_o,1),1);
        f{1,i}.u_temp = f{1,i}.u;
    end
    
    %% 迭代过程
    tI1 = clock;
    while true
%         tI1 = clock;
%         tpp = 0;
%         rho_ori
        ADMM_tag = ADMM_tag + 1;
        
        %% 1 更新 n
        parfor i = 1:d_size
            f{1,i}.n = g{1,i}.z-f{1,i}.u;
%             tpp = tpp+1;
        end
        if b_size>d_size
            parfor i = d_size+1:b_size
                [~,q] = find(rel_mat(i,:)==1);
                f{1,i}.n = [g{1,q(1)}.z;g{1,q(2)}.z]-f{1,i}.u;
%                 tpp = tpp+1;
            end
        end
%         tI2 = clock;
%         time1 = etime(tI2,tI1)/tpp;

        %% 2 更新 x
%         t_Max = 0;
        parfor i = 1:d_size
%             tII1 = clock;
            f{1,i}.H = 2*(f{1,i}.H_o+0.5*rho*eye(size(f{1,i}.H_o,1)));
            f{1,i}.f = f{1,i}.f_o-rho*f{1,i}.n;
            f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], f{1,i}.lb_o, f{1,i}.ub_o,[],options);
            if isempty(f{1,i}.x)
                f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
            end
%             tII2 = clock;
%             if etime(tII2,tII1)>t_Max
%                 t_Max = etime(tII2,tII1);
%             end
        end
        if b_size>d_size
            parfor i = d_size+1:b_size
%                 tII1 = clock;
                f{1,i}.H = 2*(f{1,i}.H_o+0.5*rho*eye(size(f{1,i}.H_o,1)));
                f{1,i}.f = f{1,i}.f_o-rho*f{1,i}.n;
                f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o,[],[],f{1,i}.lb_o, f{1,i}.ub_o,[],options);
                if isempty(f{1,i}.x)
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                end
%                 tII2 = clock;
%                 if etime(tII2,tII1)>t_Max
%                     t_Max = etime(tII2,tII1);
%                 end
            end
        end
%         time2 = t_Max;
        
        %% 3 更新 m
%         tI3 = clock;
        parfor i = 1:b_size
            f{1,i}.m = f{1,i}.x+f{1,i}.u;
        end
%         tI4 = clock;
%         time3 = etime(tI4,tI3)/b_size;
        %% 4 更新 z
%         t_Max2 = 0;
        parfor i = 1:d_size
%             tII1 = clock;
            pd = find(rel_mat(:,i) == 1);
            N = size(pd,1);
            sumM = zeros(size(f{1,1}.m,1),1);
            for j = 1:N
                if pd(j)<=d_size
                    sumM = sumM+f{1,pd(j)}.m;
                else
                    pdr = find(rel_mat(pd(j),:)==1);
                    if pd(1) == pdr(1)
                        sumM = sumM+f{1,pd(j)}.m(1:0.5*size(f{1,pd(j)}.m,1));
                    else
                        sumM = sumM+f{1,pd(j)}.m(0.5*size(f{1,pd(j)}.m,1)+1:end);
                    end
                end
            end      
            g{1,i}.z = sumM/N;
%             tII2 = clock;
%             if t_Max2 < etime(tII2,tII1)
%                 t_Max2 = etime(tII2, tII1);
%             end
        end
%         time4 = t_Max2;

        %% 5 更新 u
%         tI5 = clock;
        parfor i = 1:d_size
            f{1,i}.u = f{1,i}.u+(f{1,i}.x - g{1,i}.z);
        end
        if b_size>d_size
            parfor i = d_size+1:b_size
                [~,q] = find(rel_mat(i,:) == 1);
                f{1,i}.u = f{1,i}.u+(f{1,i}.x - [g{1,q(1)}.z;g{1,q(2)}.z]);
            end
        end
        
        %% 6 判断是否收敛
        %% 判断是否计算超出范围
        for i = 1:b_size
            if sum(abs(f{1,i}.x)> (f{1,i}.ub_o(1,1)+0.001)*ones(size(f{1,i}.x,1),1))~=0
                for iz = 1:d_size
                    z_out(iz,:) = g{1,iz}.z;
                end
                tag = ADMM_tag+max_step;
                break;
            end
        end
        if tag > max_step
            break;
        end
        
        %% 判断 pri_residual 是否收敛
%         tag_pri = 0;
        mat_pri = [];
        for i = 1:b_size
            pb = find(rel_mat(i,:) == 1);
            N = size(pb,2);
            if N == 1
%                 if abs(f{1,i}.x-g{1,pb}.z)<epsi_pri*ones(size(f{1,i}.x,1),1)
%                     tag_pri = tag_pri+1;
%                 end
                mat_pri = [mat_pri;f{1,i}.x-g{1,pb}.z];
            else             
%                 if abs(f{1,i}.x-[g{1,pb(1)}.z;g{1,pb(2)}.z])<epsi_pri*ones(size(f{1,i}.x,1),1)                    
%                     tag_pri = tag_pri+1;
%                 end
                qqq = f{1,i}.x-[g{1,pb(1)}.z;g{1,pb(2)}.z];
                mat_pri = [mat_pri;qqq];
            end
        end
        tag_pri = norm(mat_pri);
        %% 判断 dual_residual 是否收敛
%         tag_dual = 0;
%         dual_vec = [];
        mat_dual = [];
        for i = 1:d_size
%             if abs(g{1,i}.z-g{1,i}.z_temp)<epsi_dual*ones(size(g{1,i}.z,1),1)
%                 dual_vec = [dual_vec;g{1,i}.z-g{1,i}.z_temp];
%                 tag_dual = tag_dual+1;
%             end
            mat_dual = [mat_dual;g{1,i}.z-g{1,i}.z_temp];
        end
        tag_dual = norm(mat_dual);
%         test_save = [test_save;tag_pri,tag_dual];
%         tI6 = clock;
%         time5 = etime(tI6,tI5)/b_size;
        
        %% 终止条件
        % 收敛则终止
        if tag_pri <= epsi_pri && tag_dual <= epsi_dual
            for i = 1:d_size
                z_out(i,:) = g{1,i}.z;
            end
            tag = ADMM_tag;
%             disp(['step=',num2str(ADMM_tag)])
            break;
      % 限制步数，达到最大的迭代次数限制，则终止    
        elseif ADMM_tag>=max_step
            for i = 1:d_size
                z_out(i,:) = g{1,i}.z;
            end
            tag = ADMM_tag;
            break;
      %否则，保留下一轮迭代需要的值，不终止
        else
            
%             disp(['pri=',num2str(tag_pri)])
%             disp(['dual=',num2str(tag_dual)])
            for i = 1:b_size
                f{1,i}.u_temp = f{1,i}.u;
            end
            for i = 1:d_size
                g{1,i}.z_temp=g{1,i}.z;
            end
        end
    end
    tI2 = clock;
    time_ADMM = etime(tI2,tI1)/b_size;
end

