function Env_con_CCA_show( Env_con, veh_cell, ratio )
%ENV_CON_CCA_SHOW 此处显示有关此函数的摘要
%   此处显示详细说明
    
    plot([0 Env_con.width/ratio],[0 0],'k','linewidth',2);
    hold on
    plot([Env_con.width/ratio Env_con.width/ratio],[0 Env_con.height/ratio],'k','linewidth',2);
    plot([0 Env_con.width/ratio],[Env_con.height/ratio Env_con.height/ratio],'k','linewidth',2);
    plot([0 0],[0 Env_con.height/ratio],'k','linewidth',2);
    
    plot([10/ratio 58/ratio],[38/ratio 38/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
    plot([10/ratio 30/ratio],[30/ratio 30/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
    plot([38/ratio 58/ratio],[30/ratio 30/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
    plot([30/ratio 30/ratio],[10/ratio 30/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
    plot([38/ratio 38/ratio],[10/ratio 30/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
    plot(linspace(10/ratio, 58/ratio, 5), linspace(34/ratio, 34/ratio, 5),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
    plot(linspace(34/ratio, 34/ratio, 3), linspace(10/ratio, 30/ratio, 3),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
    
%     plot([9/ratio 9/ratio],[0/ratio Env_con.height/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
%     plot([21/ratio 21/ratio],[0/ratio Env_con.height/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
%     plot(linspace(13/ratio, 13/ratio, 5), linspace(0, Env_con.height/ratio, 5),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
%     plot(linspace(17/ratio, 17/ratio, 5), linspace(0, Env_con.height/ratio, 5),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
%     
    
    if ~isempty(Env_con.obstacle)
        for i = 1:size(Env_con.obstacle,1)-1
            plot([Env_con.obstacle(i,1)/ratio Env_con.obstacle(i+1,1)/ratio],[Env_con.obstacle(i,2)/ratio Env_con.obstacle(i+1,2)/ratio],'b','linewidth',2);
        end
    plot([Env_con.obstacle(i+1,1)/ratio Env_con.obstacle(1,1)/ratio],[Env_con.obstacle(i+1,2)/ratio Env_con.obstacle(1,2)/ratio],'b','linewidth',2);

    end
    
    color = ['r', 'g', 'c','b','m'];
    color = repmat(color, 1, 200);
    n = size(veh_cell,2);
    for i = 1:n
        plot(veh_cell{1,i}.x_ref/ratio, veh_cell{1,i}.y_ref/ratio,'color',color(i),'linewidth',1.5);
        Other_draw_circle(veh_cell{1,i}.x_now/ratio, veh_cell{1,i}.y_now/ratio, (veh_cell{1,i}.R_veh-0.05)/ratio, color(i));
        plot(veh_cell{1,i}.x_plan/ratio, veh_cell{1,i}.y_plan/ratio,'o','markersize',16*2.5*75/2.5/Env_con.width,'markeredgecolor',color(i));
    end
    axis([0 Env_con.width/ratio 0 Env_con.height/ratio]);
    axis equal
    drawnow;
    hold off

end

