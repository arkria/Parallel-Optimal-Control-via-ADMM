function Other_draw_circle( x0,y0,r,color )
%DRAW_CIRCLE 此处显示有关此函数的摘要
%   此处显示详细说明
    sita = 0:pi/20:2*pi;
    x = x0+r*cos(sita); y = y0+r*sin(sita);
    plot(x, y, 'k','linewidth',2);
    fill(x,y,color);
end

