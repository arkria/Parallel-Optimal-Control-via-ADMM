function h = Astar_h( now_state, final_state )
%ASTAR_H 此处显示有关此函数的摘要
%   此处显示详细说明
    h1 = pdist([now_state(1), now_state(2); final_state(1), final_state(2)]);
    if now_state(1) == final_state(1)
        if now_state(2) > final_state(2)
            h_angle = -pi/2;
        elseif now_state(2) < final_state(2)
            h_angle = pi/2;
        else
            h_angle = -1000;
        end
    else
        h_angle = atan((final_state(2)-now_state(2))/(final_state(1)-now_state(1)));
        if h_angle>=0 && now_state(1) > final_state(1)
            h_angle = h_angle-pi;
        elseif h_angle<0 && now_state(1) > final_state(1)
            h_angle = h_angle+pi;
        else
            h_angle = h_angle;
        end
    end
    h2 = Geo_angleCompare(h_angle, now_state(3));
    h3 = Geo_angleCompare(final_state(3), now_state(3));
    h = h1+10*h2+8*h3;
end

