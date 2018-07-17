function [z_out,tag,test_save, rho_mat, time_mat, A_set_save] = ADMM_DR_sy( f, g, rel_mat, rho, max_step, epsi_rel, epsi_abs, accuracy, empty_tag )
  
%% 参数
    accuracy = round(accuracy,5);
    options = optimoptions('quadprog','Display','off');
    over_relx = 1.6; % 超松弛参数
    tau_inc =2; %自适应参数 
    tau_dec = 2;
    mu = 10; % 倍数
    cumu = 3; % 累计数
    veh_num = size(g,2);
    if ~isempty(rel_mat)
        [c_size, p_size] = size(rel_mat);
        Np = size(g{1,1}.z_temp,1);
        z_out = zeros(p_size,Np);
        ceshi = zeros(p_size, Np);
        ADMM_tag = 1;
        tag = 0;
        test_save = [];
        rho_mat = [rho,0];
        time_mat = [];
        AA = eye((sum(sum(rel_mat)))*Np, (sum(sum(rel_mat)))*Np);
        BB = zeros((sum(sum(rel_mat)))*Np, p_size*Np);
        k = 0;
        for i = 1:c_size
            for j = 1:p_size
                if rel_mat(i,j) == 1
                    k = k + 1;
                    BB((k-1)*Np+1:(k-1)*Np+Np, (j-1)*Np+1:(j-1)*Np+Np) = eye(Np);
                end
            end
        end
        BB = -BB;

        %% asychronous 数据结构
        A_set_save = [];
        A_set = 1:c_size;
        for i = 1:p_size             
            for j = 1:g{1,i}.N
                if ismember(g{1,i}.rece_info(1,j), A_set)
                    g{1,i}.rece_mat(:,j) = f{1,g{1,i}.rece_info(1,j)}.z_temp((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)+f{1,g{1,i}.rece_info(1,j)}.lambda((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)'/rho;                        
                end                  
            end                
        end       
        while true
            %% 第一步 
            for i = 1:p_size
                g{1,i}.x = zeros(Np,1);
                for j = 1:g{1,i}.N
                    g{1,i}.x = g{1,i}.x+g{1,i}.rece_mat(:,j);
                end
                g{1,i}.z_retain = g{1,i}.z_temp;
                g{1,i}.z_temp = g{1,i}.x/g{1,i}.N;
                g{1,i}.z_temp = round(g{1,i}.z_temp, 5);
             end
            %% 数据传递 
            for i = A_set
                f{1,i}.rece_mat = [];
                for j = 1:size(f{1,i}.rece_info,2)
                    f{1,i}.rece_mat = [f{1,i}.rece_mat;over_relx*g{1,f{1,i}.rece_info(1,j)}.z_temp+(1-over_relx)*f{1,i}.z_temp((j-1)*Np+1:(j-1)*Np+Np)];
                    f{1,i}.rece_mat = round(f{1,i}.rece_mat,5);
                end
            end           
            %% 第二步
            for i = 1:c_size
                if ismember(i, A_set)
                    A_set_save(ADMM_tag, i) = 1;
                else
                    A_set_save(ADMM_tag, i) = 0;
                end
            end
            for i = A_set
                t1 = clock;
                f{1,i}.H = 2*(f{1,i}.H_o+0.5*rho*eye(f{1,i}.mat_size));
                f{1,i}.f = f{1,i}.f_o+f{1,i}.lambda'-rho*f{1,i}.rece_mat;
                if empty_tag == 0
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], f{1,i}.lb_o, f{1,i}.ub_o,[],options);
                    if isempty(f{1,i}.x)
                        disp('empty');
                        f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    end
                    f{1,i}.z_retain = f{1,i}.z_temp;
                    f{1,i}.z_temp = Other_boundFun(f{1,i}.x, [f{1,i}.lb_o, f{1,i}.ub_o]); % 将求得的值限制在上下界范围内
                    f{1,i}.z_temp = round(f{1,i}.z_temp, 5);
                else
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    f{1,i}.z_retain = f{1,i}.z_temp;
                    f{1,i}.z_temp = f{1,i}.x;
                end
                t2  =clock;
                time_mat(ADMM_tag, i) = etime(t2,t1);
            end                   
            %% 第三步
            for i = A_set
                f{1,i}.lambda = f{1,i}.lambda+rho*(f{1,i}.z_temp- f{1,i}.rece_mat)';
                f{1,i}.lambda = round(f{1,i}.lambda, 5);
            end
            %% 数据传递                       
            for i = 1:p_size             
                for j = 1:g{1,i}.N
                    if ismember(g{1,i}.rece_info(1,j), A_set)
                        g{1,i}.rece_mat(:,j) = f{1,g{1,i}.rece_info(1,j)}.z_temp((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)+f{1,g{1,i}.rece_info(1,j)}.lambda((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)'/rho;                        
                        g{1,i}.rece_mat = round(g{1,i}.rece_mat, 5);
                    end                  
                end                
            end
            %% 收敛判断
            XX = zeros((sum(sum(rel_mat)))*Np, 1);
            XX_re = zeros((sum(sum(rel_mat)))*Np, 1);
            YY = zeros((sum(sum(rel_mat)))*Np, 1);
            k = 0;
            for i = 1:c_size
                l = 0;
                for j = 1:p_size
                    if rel_mat(i,j) == 1
                        l = l+1;
                        k = k+1;
                        XX((k-1)*Np+1:(k-1)*Np+Np) = f{1,i}.z_temp((l-1)*Np+1:(l-1)*Np+Np);
                        XX_re((k-1)*Np+1:(k-1)*Np+Np) = f{1,i}.z_retain((l-1)*Np+1:(l-1)*Np+Np);
                        YY((k-1)*Np+1:(k-1)*Np+Np) = f{1,i}.lambda((l-1)*Np+1:(l-1)*Np+Np);
                    end
                end
            end
            ZZ = zeros(p_size*Np, 1);
            ZZ_re = zeros(p_size*Np, 1);
            for i =1:p_size
                ZZ((i-1)*Np+1:(i-1)*Np+Np) = g{1,i}.z_temp;
                ZZ_re((i-1)*Np+1:(i-1)*Np+Np) = g{1,i}.z_retain;
            end
            % pri_residual
            mat_pri = AA*XX+BB*ZZ;
            tag_pri = norm(mat_pri);
            % dual_residual
            mat_dual = rho*BB'*AA*(XX-XX_re);
            tag_dual = norm(mat_dual);            
            %% 终止判断
            epsi_pri = sqrt(size(BB,1))*epsi_abs+epsi_rel*max(norm(AA*XX), norm(BB*ZZ));
            epsi_dual = sqrt(size(BB,2))*epsi_abs+epsi_rel*norm(BB'*YY);
            test_save = [test_save;tag_pri,tag_dual, epsi_pri, epsi_dual];
            if tag_pri <= epsi_pri && tag_dual <= epsi_dual
                for i = 1:p_size
                    z_out(i,:) = g{1,i}.z_temp;
                end
                tag = ADMM_tag;
                break;    
            elseif ADMM_tag>=max_step
                for i = 1:p_size
                    z_out(i,:) = g{1,i}.z_temp;
                end
                tag = ADMM_tag;
                break;
            else
                for i = 1:p_size
                    ceshi(i,:) = g{1,i}.z_temp;
                end
%                 ERror = abs(sum(accuracy - ceshi)./sum(abs(accuracy)))*100;
                ERror = max(abs((accuracy - ceshi)/max(abs(accuracy)))*100); % 最大的误差
                disp(['step=',num2str(ADMM_tag)]);
                disp(['rho = ', num2str(rho)]);
                disp(['epsi_pri=',num2str(epsi_pri)]);
                disp(['pri=',num2str(tag_pri)]);
                disp(['epsi_dual=',num2str(epsi_dual)]);
                disp(['dual=',num2str(tag_dual)]);
                disp(['error(%) = ', num2str(ERror)]);
                disp('z = ');
                disp(ZZ(1:7)');
                disp('x = ');
                disp(XX(1:7)');
                disp('lambda = ');
                disp(YY(1:7)');
                disp('---------')                
%                 if tag_pri/epsi_pri>=mu*tag_dual/epsi_dual
                if tag_pri>=mu*tag_dual
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)+1];
%                 elseif tag_dual/epsi_dual>=mu*tag_pri/epsi_pri
                elseif tag_dual>=mu*tag_pri
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)-1];
                else
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)];
                end
                if rho_mat(ADMM_tag+1,2)>=cumu
                    rho = min(rho*tau_inc,10000);
                    rho_mat(ADMM_tag+1,2) = 0;
                elseif rho_mat(ADMM_tag+1,2)<=-cumu
                    rho = rho/tau_dec;
                    rho_mat(ADMM_tag+1,2) = 0;
                end
                ADMM_tag = ADMM_tag+1;       
            end
        end
    else
        tag = [];
        test_save = [];
        rho_mat = [];
        c_size = size(f,2);
        Np = size(f{1,1}.z_temp, 1);
        z_out = zeros(c_size, Np);
        for i = 1:c_size
            f{1,i}.H = 2*f{1,i}.H_o;
            f{1,i}.f = f{1,i}.f_o;
            if sum(f{1,i}.H) ~= 0
                f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], f{1,i}.lb_o, f{1,i}.ub_o,[],options);
    %             f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, [], [], [], [], [], [],[],options);
                if isempty(f{1,i}.x) % 防止线性不等式约束与上下界约束无交集，导致无解情况，去掉上下界约束重新求解
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                end
                f{1,i}.z_temp = Math_bound(f{1,i}.x, f{1,i}.lb_o(1,1), f{1,i}.ub_o(1,1)); % 将求得的值限制在上下界范围内
            else
                f{1,i}.z_temp = zeros(Np,1);
            end
            ceshi(i,:) = f{1,i}.z_temp;
        end
        for i = 1:c_size
            z_out(i,:) = f{1,i}.z_temp;
        end
        ERror = sum(accuracy - ceshi)/veh_num;
        disp(['error = ', num2str(ERror)]);
    end
end

