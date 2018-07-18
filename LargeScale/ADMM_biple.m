function [z_out,tag, test_save, rho_mat] = ADMM_biple( f, g, CTP_mat, rho, max_step, epsi_rel, epsi_abs, accuracy, empty_tag )
%ADMM1 此处显示有关此函数的摘要
%   此处显示详细说明
    options = optimoptions('quadprog','Display','off');
    veh_num = size(f,2);
    if ~isempty(CTP_mat)
        [c_size, p_size] = size(CTP_mat);
        b_size = 2*p_size;
        Np = size(f{1,1}.z_temp, 1);
        z_out = zeros(c_size, Np);
        ceshi = zeros(c_size, Np);
        ADMM_tag = 1;
        tag = 0;
        test_save = [];
        rho_mat = [rho,0];
        AA = zeros(b_size*Np, c_size*Np);
        BB = zeros(b_size*Np, b_size*Np);
        bb = 1;
        for i = 1:c_size
            if f{1,i}.per_size ~= 0
                for j = 1:f{1,i}.per_size
                    AA((bb-1)*Np+1:bb*Np, (i-1)*Np+1:i*Np) = eye(Np);
                    if i == g{1,f{1,i}.per_set(j)}.cen_set(1)
                        BB((bb-1)*Np+1:bb*Np, (f{1,i}.per_set(j)-1)*2*Np+1:(f{1,i}.per_set(j)-1)*2*Np+Np) = eye(Np);
                    else
                        BB((bb-1)*Np+1:bb*Np, (f{1,i}.per_set(j)-1)*2*Np+Np+1:f{1,i}.per_set(j)*2*Np) = eye(Np);
                    end   
                    bb = bb+1;
                end
            end
        end
        CC = AA'*BB;

        while true
            %% 第一步
            for i = 1:c_size
                H_temp = zeros(Np);
                f_temp = zeros(Np,1);
                if f{1,i}.per_size ~=0
                    for j = 1:f{1,i}.per_size
                        H_temp = H_temp+0.5*rho*eye(Np);
                        if i == g{1,f{1,i}.per_set(j)}.cen_set(1)
                            f_temp = f_temp+(f{1,i}.lambda(j,:)'-rho*g{1,f{1,i}.per_set(j)}.z_temp(1:Np));
                        else
                            f_temp = f_temp+(f{1,i}.lambda(j,:)'-rho*g{1,f{1,i}.per_set(j)}.z_temp(Np+1:2*Np));
                        end
                    end
                end
                f{1,i}.H = 2*(f{1,i}.H_o+H_temp);
                f{1,i}.f = f{1,i}.f_o+f_temp;
                if empty_tag == 0
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], f{1,i}.lb_o, f{1,i}.ub_o,[],options);
                    if isempty(f{1,i}.x)
                        f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    end
                    f{1,i}.z_temp = Other_boundFun(f{1,i}.x, [f{1,i}.lb_o, f{1,i}.ub_o]); % 将求得的值限制在上下界范围内
                else
                    f{1,i}.x = quadprog(f{1,i}.H, f{1,i}.f, f{1,i}.A_o, f{1,i}.b_o, [], [], [], [],[],options);
                    f{1,i}.z_temp = f{1,i}.x;
                end
%                 
            end
        %% 第二步
            for j = 1:p_size
                g{1,j}.H = 2*(0.5*rho*eye(2*Np));
                for p1 = 1:f{1,g{1,j}.cen_set(1)}.per_size
                    if j == f{1,g{1,j}.cen_set(1)}.per_set(p1)
                        break;
                    end
                end
                for p2 = 1:f{1,g{1,j}.cen_set(2)}.per_size
                    if j == f{1,g{1,j}.cen_set(2)}.per_set(p2)
                        break;
                    end
                end
                g{1,j}.f = -([f{1,g{1,j}.cen_set(1)}.lambda(p1,:),f{1,g{1,j}.cen_set(2)}.lambda(p2,:)]'+rho*[f{1,g{1,j}.cen_set(1)}.z_temp;f{1,g{1,j}.cen_set(2)}.z_temp]);
                if empty_tag == 0
                    g{1,j}.x = quadprog(g{1,j}.H, g{1,j}.f, g{1,j}.A_o, g{1,j}.b_o, [], [], g{1,j}.lb_o, g{1,j}.ub_o,[],options);
                    if isempty(g{1,j}.x)
                        g{1,j}.x = quadprog(g{1,j}.H, g{1,j}.f, g{1,j}.A_o, g{1,j}.b_o, [], [], [], [],[],options);
                    end
                    g{1,j}.z_retain = g{1,j}.z_temp;
                    g{1,j}.z_temp = Other_boundFun(g{1,j}.x, [g{1,j}.lb_o, g{1,j}.ub_o]);  
                else
                    g{1,j}.x = quadprog(g{1,j}.H, g{1,j}.f, g{1,j}.A_o, g{1,j}.b_o, [], [], [], [],[],options);
                    g{1,j}.z_retain = g{1,j}.z_temp;
                    g{1,j}.z_temp = g{1,j}.x;
                end

                
%                           
            end
        %% 第三步
            for i = 1:c_size
                if f{1,i}.per_size ~=0
                   for j = 1:f{1,i}.per_size
                        if i == g{1,f{1,i}.per_set(j)}.cen_set(1)
                            f{1,i}.lambda(j,:) = f{1,i}.lambda(j,:) + rho*(f{1,i}.z_temp-g{1,f{1,i}.per_set(j)}.z_temp(1:Np))';
                        else
                            f{1,i}.lambda(j,:) = f{1,i}.lambda(j,:) + rho*(f{1,i}.z_temp-g{1,f{1,i}.per_set(j)}.z_temp(Np+1:2*Np))';
                        end          
                   end 
                end

            end
          %% 收敛判断
            %% pri_residual
            tag_pri = 0;
            mat_pri = [];
            mat_x = [];
            for i = 1:c_size
                mat_x = [mat_x;f{1,i}.z_temp];
                if f{1,i}.per_size ~=0
                    for j = 1:f{1,i}.per_size

                        if i == g{1,f{1,i}.per_set(j)}.cen_set(1)
                            mat_pri = [mat_pri;f{1,i}.z_temp-g{1,f{1,i}.per_set(j)}.z_temp(1:Np)];
                        else
                            mat_pri = [mat_pri;f{1,i}.z_temp-g{1,f{1,i}.per_set(j)}.z_temp(Np+1:2*Np)];
                        end
                    end
                end
            end
            tag_pri = norm(mat_pri);
            %% dual_residual
            tag_dual = 0;
            mat_dual = [];
            mat_z = [];
            for j = 1:p_size
                mat_dual = [mat_dual;(g{1,j}.z_temp-g{1,j}.z_retain)];
                mat_z = [mat_z;g{1,j}.z_temp];
            end
            tag_dual = norm(rho*CC*mat_dual);
            test_save = [test_save;tag_pri,tag_dual];
            %% 终止判断
            epsi_pri = sqrt(b_size*Np)*epsi_abs+epsi_rel*max(norm(AA*mat_x), norm(BB*mat_z));
            epsi_dual = sqrt(c_size*Np)*epsi_abs+epsi_rel*norm(AA'*mat_z);
            
            if tag_pri<= epsi_pri && tag_dual <= epsi_dual 
                for i = 1:c_size
                    if empty_tag == 0
                        z_out(i,:) = f{1,i}.z_temp;
                    else
                        z_out(i,:) = Other_boundFun(f{1,i}.z_temp, [f{1,i}.lb_o, f{1,i}.ub_o]);
                    end
                    
                end
                tag = ADMM_tag;
                disp(['step=',num2str(ADMM_tag)]);
                break;
            elseif ADMM_tag>=max_step
                for i = 1:c_size
                    if empty_tag == 0
                        z_out(i,:) = f{1,i}.z_temp;
                    else
                        z_out(i,:) = Other_boundFun(f{1,i}.z_temp, [f{1,i}.lb_o, f{1,i}.ub_o]);
                    end
                end
                tag = ADMM_tag;
                break;
            else
                for i = 1:c_size
                    ceshi(i,:) = Other_boundFun(f{1,i}.z_temp, [f{1,i}.lb_o, f{1,i}.ub_o]);
                end
                ERror = sum(accuracy - ceshi)/veh_num;
                disp(['step=',num2str(ADMM_tag)]);
                disp(['rho = ', num2str(rho)]);
                disp(['epsi_pri=',num2str(epsi_pri)]);
                disp(['pri=',num2str(tag_pri)]);
                disp(['epsi_dual=',num2str(epsi_dual)]);
                disp(['dual=',num2str(tag_dual)]);
                disp(['error = ', num2str(ERror)]);

%                 disp('---------')
                pause(0.1);
    %   % 取消注释可以变为自适应算法
                tau_inc = 4;
                tau_dec = 4;
                mu = 1.7;
                cumu = 5;
                if tag_pri>=mu*tag_dual
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)+1];
                elseif tag_dual>=mu*tag_pri
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)-1];
                else
                    rho_mat(ADMM_tag+1,:) = [rho,rho_mat(ADMM_tag,2)];
                end
                if rho_mat(ADMM_tag+1,2)>=cumu
                    rho = rho*tau_inc;
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

