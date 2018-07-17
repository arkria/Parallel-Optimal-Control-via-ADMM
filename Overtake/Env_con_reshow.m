function Env_con_reshow( Env_con, nowstate, goal, final_tar )
%ENV_CON_RESHOW 此处显示有关此函数的摘要
%   此处显示详细说明
    
    plot([0 Env_con.width],[0 0],'k','linewidth',2);
    hold on
    plot([Env_con.width Env_con.width],[0 Env_con.height],'k','linewidth',2);
    plot([0 Env_con.width],[Env_con.height Env_con.height],'k','linewidth',2);
    plot([0 0],[0 Env_con.height],'k','linewidth',2);
    if ~isempty(Env_con.obstacle)
        for i = 1:size(Env_con.obstacle,1)-1
            plot([Env_con.obstacle(i,1) Env_con.obstacle(i+1,1)],[Env_con.obstacle(i,2) Env_con.obstacle(i+1,2)],'b','linewidth',2);
        end
    plot([Env_con.obstacle(i+1,1) Env_con.obstacle(1,1)],[Env_con.obstacle(i+1,2) Env_con.obstacle(1,2)],'b','linewidth',2);

    end
    plot(goal(1),goal(2),'Marker','^','Markerfacecolor',[0.3 0.2 0.1],'Markeredgecolor','g','Markersize',10)
    plot(final_tar(1),final_tar(2),'Marker','p','Markerfacecolor',[0.7 0.8 0.1],'Markeredgecolor','r','Markersize',10)
    plot(nowstate(1),nowstate(2),'Marker','o','Markerfacecolor','red','Markeredgecolor','g','Markersize',10)
    axis([0 Env_con.width 0 Env_con.height]);
    axis equal
    drawnow;
    hold off
end

