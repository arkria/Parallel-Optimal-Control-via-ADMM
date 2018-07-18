function [ veh_struct ] = Start_judge_CCA( veh_struct, time_flag )
%START_JUDGE_CCA 此处显示有关此函数的摘要
%   此处显示详细说明
    if veh_struct.start_time > time_flag
        veh_struct.v =0;
        veh_struct.start = 0;
    else
        veh_struct.v = veh_struct.v_temp;
        veh_struct.start = 1;
    end


end

