
num = size(Veh_group,2);
for i = 1:num
    Veh_group{1,i}.real_path = [repmat(Veh_group{1,i}.real_path(1,:),10,1);Veh_group{1,i}.real_path];
end