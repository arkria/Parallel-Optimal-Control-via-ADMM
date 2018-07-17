function Result_video( Env_con, veh_cell, dwell, tail, ratio, filename )
%ENV_CON_CCA_SHOW 此处显示有关此函数的摘要
%   此处显示详细说明
    time_end = size(veh_cell{1,1}.real_path,1);
%     AA(1:time_end) = struct('cdata',[],'colormap',[]);
    videoObj = VideoWriter([filename, '.avi'],'Uncompressed AVI');
    videoObj.FrameRate = 0.4/dwell;
    open(videoObj);
    V_L = 3.5; V_W = 1.75;
%     tail = 100;
    for tt = 1:time_end
        plot([0 Env_con.width/ratio],[0 0],'k','linewidth',2);
        hold on
        plot([Env_con.width/ratio Env_con.width/ratio],[0 Env_con.height/ratio],'k','linewidth',2);
        plot([0 Env_con.width/ratio],[Env_con.height/ratio Env_con.height/ratio],'k','linewidth',2);
        plot([0 0],[0 Env_con.height/ratio],'k','linewidth',2);
        plot([9/ratio 9/ratio],[0/ratio Env_con.height/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
        plot([21/ratio 21/ratio],[0/ratio Env_con.height/ratio],'color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',2);
        plot(linspace(13/ratio, 13/ratio, 5), linspace(0, Env_con.height/ratio, 5),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
        plot(linspace(17/ratio, 17/ratio, 5), linspace(0, Env_con.height/ratio, 5),'--','color',[hex2dec('6c')/255, hex2dec('7b')/255, hex2dec('8b')/255],'linewidth',1.5);
 
        if ~isempty(Env_con.obstacle)
            for i = 1:size(Env_con.obstacle,1)-1
                plot([Env_con.obstacle(i,1)/ratio Env_con.obstacle(i+1,1)/ratio],[Env_con.obstacle(i,2)/ratio Env_con.obstacle(i+1,2)/ratio],'b','linewidth',2);
            end
        plot([Env_con.obstacle(i+1,1)/ratio Env_con.obstacle(1,1)/ratio],[Env_con.obstacle(i+1,2)/ratio Env_con.obstacle(1,2)/ratio],'b','linewidth',2);

        end

        color = ['r', 'g', 'b', 'c', 'm'];
        color = repmat(color,1,20);
        tail_color = zeros(tail, 3);
        color_end = [hex2dec('ff')/255, hex2dec('00')/255, hex2dec('ff')/255];
        color_start = [hex2dec('00')/255, hex2dec('00')/255, hex2dec('ff')/255];
        tail_color(:,1) = linspace(color_start(1), color_end(1), tail);
        tail_color(:,2) = linspace(color_start(2), color_end(2), tail);
        tail_color(:,3) = linspace(color_start(3), color_end(3), tail);
        marker = ['o', 's','^','s','o','s','o','s','o','s'];
        n = size(veh_cell,2);
        for i = 1:n
            if tt <= tail
                plot(veh_cell{1,i}.real_path(1:tt,1)/ratio, veh_cell{1,i}.real_path(1:tt,2)/ratio,'color',color(i),'linewidth',1.5);
                for ttt = 1:2:tt
                    plot(veh_cell{1,i}.real_path(ttt,1)/ratio, veh_cell{1,i}.real_path(ttt,2)/ratio,'marker', marker(i),'markeredgecolor', tail_color(tail-tt+ttt,:),'markerfacecolor', tail_color(tail-tt+ttt,:),'markersize',7);
                end
            else
                plot(veh_cell{1,i}.real_path(tt-tail:tt,1)/ratio, veh_cell{1,i}.real_path(tt-tail:tt,2)/ratio,'color',color(i),'linewidth',1.5);
                for ttt = tt-tail+1:2:tt
                    plot(veh_cell{1,i}.real_path(ttt,1)/ratio, veh_cell{1,i}.real_path(ttt,2)/ratio,'marker', marker(i),'markeredgecolor', tail_color(ttt-tt+tail,:),'markerfacecolor', tail_color(tail-tt+ttt,:),'markersize',7);
                end
                 
            end
            center_x = veh_cell{1,i}.real_path(tt,1)/ratio+((V_L-veh_cell{1,i}.L)*0.5)*cos(veh_cell{1,i}.real_path(tt,3));
            center_y = veh_cell{1,i}.real_path(tt,2)/ratio+((V_L-veh_cell{1,i}.L)*0.5)*sin(veh_cell{1,i}.real_path(tt,3));
            Other_draw_reactangle(V_L, V_W, [center_x, center_y], veh_cell{1,i}.real_path(tt,3), color(i), 4);
%             Other_draw_circle(veh_cell{1,i}.real_path(tt,1)/ratio, veh_cell{1,i}.real_path(tt,2)/ratio, (veh_cell{1,i}.R_veh-0.2)/ratio, color(i));
%             plot(veh_cell{1,i}.x_plan, veh_cell{1,i}.y_plan,'o','markersize',16,'markeredgecolor',color(i));
        end
%         axis([0 Env_con.width/ratio 0 Env_con.height/ratio]);
        axis([8 22 80 170]);
        axis equal
        drawnow;
        pause(dwell)
        hold off
        disp(['step = ', num2str(tt)]);
%         AA(tt) = getframe;
        frame = getframe;
        writeVideo(videoObj, frame);
    end
%     movie2avi(AA,'movie','compression','none','quality', 100);
    close(videoObj);

end

