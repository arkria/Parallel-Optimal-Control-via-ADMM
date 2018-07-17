figure(1)
hold on
for i = 1:veh_n
    plot(time_mat(:,i), '-o');
%     pause(1)
%     drawnow
end
sum_n = size(time_mat,2);
for i = veh_n+1:sum_n
    plot(time_mat(:,i), '-s');
%     pause(1)
%     drawnow
end