function Env_struct = Env_con( height, width, road_width, obstacle, res )
    % height为场景高度，width为场景宽度，obstacle为场景中的障碍物信息，为一个n*2的数组；res为场景的分辨率 
    Env_struct.height = height;
    Env_struct.width = width;
    Env_struct.obstacle = obstacle;
    Env_struct.res = res;
    Env_struct.road_width = road_width;
end

