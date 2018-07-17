function score = Astar_dist( now_state, next_state, delta )
%ASTAR_DIST 此处显示有关此函数的摘要
%   此处显示详细说明
    w1 = 1; w2 = 0.1;
    Tn = pdist([now_state(1:2);next_state(1:2)]);
    An = delta^2;
    score = w1*Tn+w2*An;

end

