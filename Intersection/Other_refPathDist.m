function [dist, tag] = Other_refPathDist( x_now, y_now, x_ref, y_ref )
%OTHER_REFPATHDIST 求参考轨迹中距离当前状态最近的点，返回距离(dist)以及该点在参考轨迹中的编号(tag)
%   此处显示详细说明
    n = size(x_ref,1);
    dist = 10000;tag = 0;
    for i = 1:n
        dist_temp = pdist([x_now, y_now; x_ref(i), y_ref(i)]);
        if dist_temp <= dist
            dist = dist_temp;
            tag = i;
        end
    end

end

