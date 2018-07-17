function Other_draw_reactangle( height, width, point, angle, color, linewidth )
%OTHER_DRAW_REACTANGLE 此处显示有关此函数的摘要
%   此处显示详细说明
    rd(1) = point(1)+0.5*height*cos(angle)+0.5*width*sin(angle);
    rd(2) = point(2)+0.5*height*sin(angle)-0.5*width*cos(angle);
    
    ru(1) = point(1)+0.5*height*cos(angle)-0.5*width*sin(angle);
    ru(2) = point(2)+0.5*height*sin(angle)+0.5*width*cos(angle);
    
    lu(1) = 2*point(1)-rd(1);
    lu(2) = 2*point(2)-rd(2);
    
    ld(1) = 2*point(1)-ru(1);
    ld(2) = 2*point(2)-ru(2);
    
    hold on
    plot([rd(1), ru(1)], [rd(2), ru(2)], color, 'linewidth', linewidth);
    plot([ru(1), lu(1)], [ru(2), lu(2)], color, 'linewidth', linewidth);
    plot([lu(1), ld(1)], [lu(2), ld(2)], color, 'linewidth', linewidth);
    plot([ld(1), rd(1)], [ld(2), rd(2)], color, 'linewidth', linewidth);
    axis equal
    
end

