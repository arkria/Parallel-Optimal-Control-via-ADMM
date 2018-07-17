function out_mat = Other_boundFun( in_mat, bound )
%BOUNDFUN 此处显示有关此函数的摘要
%   此处显示详细说明
    [m,n] = size(in_mat);
    out_mat = zeros(m,n);
    if ~isempty(bound)
        for i = 1:m
            for j = 1:n
                if in_mat(i,j) >= bound(i,2)
                    out_mat(i,j) = bound(i,2);
                elseif in_mat(i,j) <= bound(i,1)
                    out_mat(i,j) = bound(i,1);
                else
                    out_mat(i,j) = in_mat(i,j);
                end
            end
        end
    else
        out_mat = in_mat;
    end
    

end

