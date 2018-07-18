clear
clc
num = 7;
Contain = cell(1,num);
cut_len = 10;
for i = 1:num
    a = load(['Vg',num2str(i),'.mat']);
    Contain{1,i} = a;
    if i <= 3
        veh_n = size(Contain{1,i}.Veh_group, 2);
        for j = 1:veh_n
            Contain{1,i}.Veh_group{1,j}.real_path(1:(i-1)*cut_len, :) = [];
        end
    end
    
    
end



k = 0;
temp = Contain{1,num}.Veh_group{1,1};
time_len = size(temp.real_path,1);
for i = 1:num
    veh_n = size(Contain{1,i}.Veh_group, 2);
    for j = 1:veh_n
        k = k+1;
        Veh_group{1,k} = Contain{1,i}.Veh_group{1,j};
        if size(Veh_group{1,k}.real_path,1) < time_len
            Veh_group{1,k}.real_path(end+1:time_len,:) = repmat(Veh_group{1,k}.real_path(end,:),time_len-size(Veh_group{1,k}.real_path,1), 1);
        end
    end
end