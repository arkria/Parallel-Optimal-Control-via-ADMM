function [z_out,tag,test_save, rho_mat, time_mat, A_set_save] = ADMM_DR_asy( f, g, rel_mat, rho, max_step, epsi_rel, epsi_abs, accuracy, empty_tag )
  
    options = optimoptions('quadprog','Display','off');
    veh_num = size(g,2);
    if ~isempty(rel_mat)
        [c_size, p_size] = size(rel_mat);
        Np = size(f{1,1}.z_temp,1);
        z_out = zeros(p_size,Np);
        ceshi = zeros(p_size, Np);
        ADMM_tag = 1;
        tag = 0;
        test_save = [];
        rho_mat = [rho,0];
        time_mat = [];
        AA = eye((p_size+2*(c_size-p_size))*Np, (p_size+2*(c_size-p_size))*Np);
        BB = zeros((p_size+2*(c_size-p_size))*Np, p_size*Np);
        for i = 1:c_size
            if i <= p_size
                BB((i-1)*Np+1:(i-1)*Np+Np, (i-1)*Np+1:(i-1)*Np+Np) = eye(Np);
            else
                pd = find(rel_mat(i,:) == 1);
                BB(p_size*Np+(i-p_size-1)*2*Np+1:p_size*Np+(i-p_size-1)*2*Np+Np, (pd(1)-1)*Np+1:(pd(1)-1)*Np+Np) = eye(Np);
                BB(p_size*Np+(i-p_size-1)*2*Np+Np+1:p_size*Np+(i-p_size-1)*2*Np+2*Np, (pd(2)-1)*Np+1:(pd(2)-1)*Np+Np) = eye(Np);
            end
        end
        BB = -BB;

        %% asychronous 数据结构
        A_set_save = [];
        thres_time = 1;
%         thres_time = 0.0051;
        thres_step = 2;
        ALL_set = 1:c_size;
        A_set = 1:c_size;
        C_set = [];
        newC_set = [];
        for i = A_set
            if i<=p_size
                f{1,i}.rece_mat = g{1,f{1,i}.rece_info(1,1)}.z_temp;
            else
                f{1,i}.rece_mat = [g{1,f{1,i}.rece_info(1,1)}.z_temp;g{1,f{1,i}.rece_info(1,2)}.z_temp];
            end
        end
        for i = 1:p_size             
            for j = 1:g{1,i}.N
                if ismember(g{1,i}.rece_info(1,j), A_set)
                    g{1,i}.rece_mat(:,j) = f{1,g{1,i}.rece_info(1,j)}.z_temp((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)+f{1,g{1,i}.rece_info(1,j)}.lambda((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)'/rho;                        
                end                  
            end                
        end  
        rand_tag = 0;
        over_relx = 1.7;
        while true
             for i = 1:p_size
                g{1,i}.x = zeros(Np,1);
                for j = 1:g{1,i}.N
                    g{1,i}.x = g{1,i}.x+g{1,i}.rece_mat(:,j);
                end
                g{1,i}.z_retain = g{1,i}.z_temp;
                g{1,i}.z_temp = g{1,i}.x/g{1,i}.N;
             end
            %% 数据传递 
            for i = A_set
                if i<=p_size
                    f{1,i}.rece_mat = over_relx*g{1,f{1,i}.rece_info(1,1)}.z_temp+(1-over_relx)*f{1,i}.z_temp;
                else
                    f{1,i}.rece_mat = [over_relx*g{1,f{1,i}.rece_info(1,1)}.z_temp+(1-over_relx)*f{1,i}.z_temp(1:Np);over_relx*g{1,f{1,i}.rece_info(1,2)}.z_temp+(1-over_relx)*f{1,i}.z_temp(Np+1:2*Np)];
                end
            end           
            %% 第一步 ,,ll
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
                        f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    end
                    f{1,i}.z_retain = f{1,i}.z_temp;
                    f{1,i}.z_temp = Other_boundFun(f{1,i}.x, [f{1,i}.lb_o, f{1,i}.ub_o]); % 将求得的值限制在上下界范围内
                else
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    f{1,i}.z_retain = f{1,i}.z_temp;
                    f{1,i}.z_temp = f{1,i}.x;
                end
                t2  =clock;
                time_mat(ADMM_tag, i) = etime(t2,t1);
                if time_mat(ADMM_tag, i)> thres_time
                    newC_set = [newC_set, [i; min(ceil(time_mat(ADMM_tag, i)/thres_time), thres_step)]];
                end
            end         
            %% 第二步
           
            %% 第三步
            for i = A_set
                f{1,i}.lambda = f{1,i}.lambda+rho*(f{1,i}.z_temp- f{1,i}.rece_mat)';
            end
               %% 数据传递
            C_set = [];
            for i = 1:size(newC_set, 2)
                if newC_set(2,i) >= 2
                    C_set = [C_set, newC_set(1,i)];
                end
                newC_set(2,i) = newC_set(2,i)-1;
            end
            if ~isempty(newC_set)
                newC_set(:,newC_set(2,:) == 0) = [];
            end
%             A_set = ALL_set;
            if rand_tag == 0
                BA = randperm(c_size);
                A_set = BA(rand_tag+1);
                rand_tag = rand_tag+1;
            elseif rand_tag < c_size-10
                A_set = BA(rand_tag+1);
                rand_tag = rand_tag+1;
            else
                BA = randperm(c_size);
                rand_tag = 0;
                A_set = BA(rand_tag+1);
                rand_tag = 1;
            end
%             
            for i = 1:p_size             
                for j = 1:g{1,i}.N
                    if ismember(g{1,i}.rece_info(1,j), A_set)
                        g{1,i}.rece_mat(:,j) = f{1,g{1,i}.rece_info(1,j)}.z_temp((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)+f{1,g{1,i}.rece_info(1,j)}.lambda((g{1,i}.rece_info(2,j)-1)*Np+1:(g{1,i}.rece_info(2,j)-1)*Np+Np)'/rho;                        
                    end                  
                end                
            end
            %% 收敛判断
            XX = zeros((p_size+2*(c_size-p_size))*Np, 1);
            XX_re = zeros((p_size+2*(c_size-p_size))*Np, 1);
            YY = zeros((p_size+2*(c_size-p_size))*Np, 1);
            for i = 1:c_size
                if i <= p_size
                    XX((i-1)*Np+1:(i-1)*Np+Np) = f{1,i}.z_temp;
                    XX_re((i-1)*Np+1:(i-1)*Np+Np) = f{1,i}.z_retain;
                    YY((i-1)*Np+1:(i-1)*Np+Np) = f{1,i}.lambda;
                else
                    XX(p_size*Np+(i-p_size-1)*2*Np+1:p_size*Np+(i-p_size-1)*2*Np+2*Np) = f{1,i}.z_temp;
                    XX_re(p_size*Np+(i-p_size-1)*2*Np+1:p_size*Np+(i-p_size-1)*2*Np+2*Np) = f{1,i}.z_retain;
                    YY(p_size*Np+(i-p_size-1)*2*Np+1:p_size*Np+(i-p_size-1)*2*Np+2*Np) = f{1,i}.lambda;
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
            epsi_pri = sqrt((p_size+2*(c_size-p_size))*Np)*epsi_abs+epsi_rel*max(norm(AA*XX), norm(BB*ZZ));
            epsi_dual = sqrt((p_size+2*(c_size-p_size))*Np)*epsi_abs+epsi_rel*norm(BB'*YY);
            test_save = [test_save;tag_pri,tag_dual, epsi_pri, epsi_dual];
            if tag_pri <= epsi_pri && tag_dual <= epsi_dual
                for i = 1:p_size
                    z_out(i,:) = g{1,i}.z_temp;
                end
                tag = ADMM_tag;
    %             disp(['step=',num2str(ADMM_tag)])
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
                ERror = abs(sum(accuracy - ceshi)./sum(abs(accuracy)))*100;
                disp(['step=',num2str(ADMM_tag)]);
                disp(['rho = ', num2str(rho)]);
                disp(['epsi_pri=',num2str(epsi_pri)]);
                disp(['pri=',num2str(tag_pri)]);
                disp(['epsi_dual=',num2str(epsi_dual)]);
                disp(['dual=',num2str(tag_dual)]);
                disp(['error(%) = ', num2str(ERror)]);
                disp('---------')
                tau_inc =1.2;
                tau_dec = 1.2;
                mu = 1.2;
                cumu = 5;
                if tag_pri/epsi_pri>=mu*tag_dual/epsi_dual
%                 if 1>=mu*tag_dual/epsi_dual
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)+1];
                elseif tag_dual/epsi_dual>=mu*tag_pri/epsi_pri
%                 elseif 1>=mu*tag_pri/epsi_pri
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

