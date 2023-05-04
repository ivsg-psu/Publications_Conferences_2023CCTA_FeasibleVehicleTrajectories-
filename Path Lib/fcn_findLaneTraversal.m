function [start_lane, end_lane] = fcn_findLaneTraversal(lane_start,lane_end,...
    RT_LCL_lane1_new_stations,RT_LCL_lane2_new_stations,RT_LCL_lane3_new_stations,RT_LCL_lane4_new_stations)

if lane_start == 1
    start_lane = RT_LCL_lane1_new_stations;
elseif lane_start == 2
    start_lane = RT_LCL_lane2_new_stations;
elseif lane_start == 3
    start_lane = RT_LCL_lane3_new_stations;
elseif lane_start == 4
    start_lane = RT_LCL_lane4_new_stations;
end

if lane_end == 1
    end_lane = RT_LCL_lane1_new_stations;
elseif lane_end == 2
    end_lane = RT_LCL_lane2_new_stations;
elseif lane_end == 3
    end_lane = RT_LCL_lane3_new_stations;
elseif lane_end == 4
    end_lane = RT_LCL_lane4_new_stations;
end

end