function [final_path, final_action] = Astar_new( veh_struct, Env_struct, x_goal, y_goal, theta_goal )
%ASTAR 此处显示有关此函数的摘要
    obstacle = Env_struct.obstacle;
    closedset = [];
    closed_size = 0;
    openset = [veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now];
    openset_size = 1;
    came_from = containers.Map;
    g_score = containers.Map;
    h_score = containers.Map;
    f_score = containers.Map;
    g_score(num2str([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now])) = 0;
    h_score(num2str([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now])) = Astar_h([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now],[x_goal,y_goal,theta_goal]);
    f_score(num2str([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now])) = g_score(num2str([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now])) + h_score(num2str([veh_struct.x_now,veh_struct.y_now,veh_struct.theta_now]));
    while ~isempty(openset)
        op_key = openset;
%         op_value = openset.values;
        min_value = f_score(num2str(op_key(1,:)));
        min_key = op_key(1,:);
        del_key = 1;
        
        for i = 1:size(openset,1)
            if f_score(num2str(op_key(i,:))) < min_value
                min_value = f_score(num2str(op_key(i,:)));
                min_key = op_key(i,:);
                del_key = i;
            end
        end
        now_state = min_key;
        %%
        now_state %用于显示当前进行到何种状态
        dist = pdist([now_state(1:2);x_goal,y_goal])
        figure(1);
        Env_con_reshow(Env_struct, now_state, [x_goal, y_goal], [veh_struct.x_tar, veh_struct.y_tar])
%%
        if pdist([now_state(1:2);x_goal,y_goal])<veh_struct.v*veh_struct.deltaT+0.1
%         if pdist([now_state(1:2);x_goal,y_goal]) < 0.3
            [final_path, final_action] = Astar_reconstruct_path(came_from, min_key, veh_struct); % 待补充
            break
        end
        openset(del_key,:)=[];
        openset_size = openset_size-1;
        closedset(closed_size+1,:) = min_key;
        closed_size = closed_size+1;
        action_size = size(veh_struct.deltaSet,2);
        neighbor = [];
        ii = 1;
        for i = 1:action_size 
            [nei_x,nei_y,nei_theta] = veh_con_model(now_state(1), now_state(2), now_state(3), veh_struct, veh_struct.deltaSet(i));
            if Env_con_obscheck(Env_struct, [nei_x, nei_y]) == false
                neighbor(ii,:) = [nei_x,nei_y,nei_theta];
                ii = ii + 1;
            end
        end
        for i = 1:ii-1
            if ismember(neighbor(i,:), closedset,'rows')
                continue
            end
            tentative_g_score = g_score(num2str(min_key))+Astar_dist(min_key, neighbor(i,:), veh_struct.deltaSet(i));
            if ~ismember(neighbor(i,:), openset,'rows')
                openset(openset_size+1,:) = neighbor(i,:);
                openset_size = openset_size+1;
                tentative_is_better = true;
            elseif tentative_g_score < g_score(num2str(neighbor(i,:)))
                tentative_is_better = true;
            else
                tentative_is_better = false;
            end
            if tentative_is_better == true
                came_from(num2str(neighbor(i,:))) = min_key;
                g_score(num2str(neighbor(i,:))) = tentative_g_score;
                neighbor_state = neighbor(i,:);
                h_score(num2str(neighbor_state)) = Astar_h(neighbor(i,:),[x_goal,y_goal,theta_goal]);
                f_score(num2str(neighbor_state)) = g_score(num2str(neighbor_state)) + h_score(num2str(neighbor_state));
            end
        end
    end


end