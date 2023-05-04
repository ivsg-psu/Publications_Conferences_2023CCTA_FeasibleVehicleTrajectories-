% script_test_fcn_calculateSmoothLaneChange.m
% test the lane change function
%% Add paths
addpath('./Utilities')
addpath('./Utilities/Path Lib')

%% Create the lanes and path to follow
lane_1_path = [0 0; 200 0];
lane_1_traversal = fcn_Path_convertPathToTraversalStructure(lane_1_path);

lane_2_path = [0 3; 200 3];
lane_2_traversal = fcn_Path_convertPathToTraversalStructure(lane_2_path);

reference_path = [0 0; 100 0; 100 3; 200 3];
reference_traversal = fcn_Path_convertPathToTraversalStructure(reference_path);

%% Redecimate the lane and path stations at a given interval
interval = 1;
new_stations = (0:interval:200)';

lane_1_traversal = fcn_Path_newTraversalByStationResampling(lane_1_traversal,new_stations);
lane_2_traversal = fcn_Path_newTraversalByStationResampling(lane_2_traversal,new_stations);
reference_traversal = fcn_Path_newTraversalByStationResampling(reference_traversal,new_stations);

reference_path = [reference_traversal.X reference_traversal.Y]; % separate path variable
%% Calculate lane change smoothing coefficients
speed_limit = 24.444; % [m/s]
t0 = 0;
tf = 1;
s0 = 0;
% lane change takes 3 seconds on average, sf = posted_velocity*3(speed limit)
sf = speed_limit*3;

% convert coefficients to cubic equation
[a,b,c,d] = fcn_RoadSeg_calculateRoadCubicPolyCoefs(t0,tf,s0,sf);

%% Calculate the new smoothed path
index_lane_change = 101;

% create the index for where to start and end the LC on the
%   path (LC discontinuity should be in the middle)
lane_change_start_station = reference_traversal.Station(index_lane_change)-0.5*sf;
lane_change_end_station = reference_traversal.Station(index_lane_change)+0.5*sf;

% calculate station values at which the LC occurs
lane_change_stations = (lane_change_start_station:1:lane_change_end_station)';

% determine the lane the vehicle starts and ends in
start_lane = lane_1_traversal;
end_lane = lane_2_traversal;

% find the indices for before, during, and after the LC happens
indices_LC_before = reference_traversal.Station<lane_change_stations(1);
indices_LC = reference_traversal.Station>=lane_change_stations(1) ...
    & reference_traversal.Station<=lane_change_stations(end);
indices_LC_after = reference_traversal.Station>(lane_change_stations(end));
                
% smooth the lane change
reference_path_smooth = fcn_calculateSmoothLaneChange(a,b,c,d,...
                        lane_change_stations,start_lane,end_lane,reference_path,...
                        indices_LC,indices_LC_before,indices_LC_after);

%% Plot the results
figure(1)
plot(reference_path_smooth(:,1),reference_path_smooth(:,2),'b.')
ylim([-1 4])
grid on
xlabel('East [m]','FontSize',14)
ylabel('North [m]','FontSize',14)

