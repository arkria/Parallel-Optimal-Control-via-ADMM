function delta = Geo_angleCompare( alpha, beta )
%GEO_ANGLECOMPARE 在(-pi,pi]的角度范围内，求出alpha与beta的最小夹角delta（一定不会大于pi）
%   此处显示详细说明
    delta = pi;
    if alpha > beta
        large_angle = alpha;
        small_angle = beta;
    elseif alpha < beta
        large_angle = beta;
        small_angle = alpha;
    else
        delta = 0;
    end
    if delta ~= 0
        diff_angle = large_angle - small_angle;
        if diff_angle <= pi
            delta = diff_angle;
        else
            delta = 2*pi - diff_angle;
        end
    end
end

