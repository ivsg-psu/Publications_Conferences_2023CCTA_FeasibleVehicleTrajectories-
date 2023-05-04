%%%%%%%%%% script_estimateAccelForSmoothTrafficTrajSCE_3DoFPPC.m %%%%%%%%%%
%% Purpose:
% The pupose of this script create feasible reference trajectories to 
%   estimate friction demand by letting a 3DOF vehicle model track the 
%   trajectory, simulated by a microscopic traffic simulations in Aimsun.
%
% Author: Juliette Mitrovich, adapted from Satya Prasad
% Created: 2022/04/09
% Updated: 2023/03/30

%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% set this variables if flag.loadTestSectionIds is 'true'
section_ID = 3905; % set the specific section ID you want to look at
vehicle_ID = 2; % set the specific vehicle ID you want to look at

% flag triggers
flag.plot     = true; % set to 'true' to plot

%% Add path to dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities')
addpath('./Utilities/Path Lib');
addpath('./Utilities/UTM Lib');

% Global variables for the vehicle model
% this needs to be changed to pass through all functions
global flag_update global_acceleration

%% Set system parameters
deltaT           = 0.05; % Vehicle simulation step-size
aimsun_step_size = 0.1; % Microscopic simulation step-size

cut_off_length = 20; % how much data to cut off the trajectory
station_interval_RCL = 1; % interval to redecimate the RCL stations

%% Add necessary .mat files to the workspace
load A.mat % geotiff file used to add elevation to coordinates
load sections_shape.mat % shape file used to find section IDs of traffic simulation
load raw_trajectory_test_cases.mat

%% Define vehicle and controller properties
% Define a MATLAB structure that specifies the physical values for a vehicle.
% For convenience, we ask that you call this stucture 'vehicle'.
vehicle.m   = 1600; % mass (kg)
vehicle.Izz = 2500; % mass moment of inertia (kg m^2)
vehicle.Iw  = 1.2; % mass moment of inertia of a wheel (kg m^2)
vehicle.Re  = 0.32; % effective radius of a wheel (m)
vehicle.a   = 1.3; % length from front axle to CG (m)
vehicle.L   = 2.6; % wheelbase (m)
vehicle.b   = 1.3; % length from rear axle to CG (m)
vehicle.d   = 1.5; % track width (m)
vehicle.h_cg = 0.42; % height of the cg (m)
vehicle.Ca  = [95000; 95000; 110000; 110000]; % wheel cornering stiffnesses
vehicle.Cx  = [65000; 65000; 65000; 65000]; % longitudinal stiffnesses

vehicle.contact_patch_length = 0.15; % [meters]
vehicle.friction_ratio       = 1; % [No units]

controller.look_ahead_distance = 10; % look-ahead distance [meters]
controller.steering_Pgain      = 0.1; % P gain for steering control
controller.velocity_Pgain      = 0.011; % P gain for speed control

% Parameters and initial conditions for simulink and matlab model
road_properties.grade = 0;
road_properties.bank_angle = 0; % road properties
friction_coefficient  = 1.0*ones(4,1);

%% Define load transfer conditions
vdParam.longitudinalTransfer = 1;
if vdParam.longitudinalTransfer
    vdParam.lateralTransfer = 1;
    type_of_transfer = 'both';
else
    vdParam.lateralTransfer = 0;
    type_of_transfer = 'default';
end

%% Set coordinates of local origin for converting LLA to ENU
% link to repo where I got these
lat0 = 40.79365087;
lon0 = -77.86460284;
h0 = 334.719;
wgs84 = wgs84Ellipsoid;

%% Create an elevation map and convert lat and lon coordinates to UTM
% the elevation map will be used to add elevation to the road centerline
% and the trajectories
% [X,Y,elevation_map] = fcn_createElevationMapAndConvertLL2UTM(A);
load SCE_geotiff_X.mat
load SCE_geotiff_Y.mat
load SCE_geotiff_elevation_map.mat

%% Get the road centerline  data from AIMSUN shape file
% centerline data for road SECTIONS
%   [sectionID number_of_lanes X Y speed_limit]
RT_RCL_UTM_sections = fcn_calculateRoadCenterlineSection(sections_shape);

%% Convert road centerline position from UTM to ENU
RT_RCL_path_all = RT_RCL_UTM_sections(:,[3,4]);

[RT_RCL_cg_east, RT_RCL_cg_north, RT_RCL_cg_up] = ...
    fcn_addElevationAndConvertUTM2ENU...
    (RT_RCL_path_all,X,Y,elevation_map,lat0,lon0,h0,wgs84);

sectionID = RT_RCL_UTM_sections(:,1);
number_of_lanes = RT_RCL_UTM_sections(:,2);
speed_limit = RT_RCL_UTM_sections(:,5);
% concatinate needed RCL data
%   [section ID, # of lanes, cg_east, cg_north, cg_up, speed_limit]
RT_RCL_ENU_sections = ...
    [sectionID number_of_lanes RT_RCL_cg_east RT_RCL_cg_north RT_RCL_cg_up speed_limit];

%% Create the reference trajectory for the specified section ID
list_of_sectionIds = unique(raw_trajectory_test_cases{:,{'section_id'}});

N_sectionID = length(list_of_sectionIds);
sectionId = find(list_of_sectionIds == section_ID);

for index_sectionID = sectionId

    % parse out the specific section ID trajectories from the bigger mat
    % file
    indices_raw_traj_sectionId = raw_trajectory_test_cases{:,'section_id'} == list_of_sectionIds(index_sectionID);
    raw_trajectory = raw_trajectory_test_cases(indices_raw_traj_sectionId,:);
    vehicleIds_on_sectionId = unique(raw_trajectory{:,'vehicle_id'});

    %% Add elevation to the State College road-network and convert to ENU
    RT_veh_path_all = raw_trajectory{:,{'position_front_x','position_front_y'}};

    [RT_veh_cg_east, RT_veh_cg_north, RT_veh_cg_up] = fcn_addElevationAndConvertUTM2ENU...
        (RT_veh_path_all,X,Y,elevation_map,lat0,lon0,h0,wgs84);

    % change UTM coordinates in raw trajectory to new ENU coordinates
    raw_trajectory{:,{'position_front_x'}} = RT_veh_cg_east;
    raw_trajectory{:,{'position_front_y'}} = RT_veh_cg_north;

    %% Calculate lane centerline (LCL) from RCL for the specific section ID
    % Find the indices that correspond to the section ID
    indices_RCL_sectionID = RT_RCL_ENU_sections(:,1) == list_of_sectionIds(index_sectionID);
    RT_RCL_ENU_sectionID = RT_RCL_ENU_sections(indices_RCL_sectionID,:);

    % Create individual variables from RT_RCL_ENU_sectionID
    RT_RCL_path = RT_RCL_ENU_sectionID(:,[3,4]); % path [X Y]
    RT_RCL_speed_limit_kmh = unique(RT_RCL_ENU_sectionID(:,6)); % speed limit [km/h]
    RT_RCL_speed_limit_ms = RT_RCL_speed_limit_kmh/3.6; % speed limit [m/s]
    
    % Calculate the stations 
    RT_RCL_diff_station = sqrt(sum(diff(RT_RCL_path).^2,2));
    RT_RCL_station = cumsum([0; RT_RCL_diff_station]);

    % Create a reference traversal structure for the RCL path
    RT_RCL_traversal.X = RT_RCL_ENU_sectionID(:,3); % x coordinate of RCL path
    RT_RCL_traversal.Y = RT_RCL_ENU_sectionID(:,4); % y coordinate of RCL path
    RT_RCL_traversal.Z = RT_RCL_ENU_sectionID(:,5); % z coordinate of RCL path
    RT_RCL_traversal.Station = RT_RCL_station; % station that was just calculated

    % Determine how many LCL need to be calculated based on the
    %   number of lanes in that section
    number_of_lanes_in_sectionID = unique(RT_RCL_ENU_sectionID(:,2));

    % Calculate the lane centerlines: [X, Y, Yaw, Station, lane #]
    % Some of these variables will be empty if there are < 4 lanes
    [RT_LCL_lane1, RT_LCL_lane2, RT_LCL_lane3, RT_LCL_lane4, RT_LCL_XY_all] = ...
        fcn_calculateLaneCenterline(RT_RCL_traversal,number_of_lanes_in_sectionID);

    %% Redecimate the RCL stations at 1-meter increments and cut-off x-meters
    RT_RCL_new_stations_traversal = fcn_calculateNewStationsAndCutoffData...
        (RT_RCL_traversal,station_interval_RCL,cut_off_length);

    %% Calculate lane change smoothing coefficients
    t0 = 0;
    tf = 1;
    s0 = 0;

    % lane change takes 3 seconds on average, sf = posted_velocity*3(speed limit)
    sf = RT_RCL_speed_limit_ms*3; 

    % convert coefficients to cubic equation
    [a,b,c,d] = fcn_RoadSeg_calculateRoadCubicPolyCoefs(t0,tf,s0,sf);
    %% Run vehicle simulation for the chosen vehicle ID
    for index_vehicle = vehicle_ID
        %% calculate the trajectory for the index_vehicle
        indices_raw_path = raw_trajectory{:,'vehicle_id'} == vehicleIds_on_sectionId(index_vehicle);
        RT_veh_path = raw_trajectory{indices_raw_path,...
            {'position_front_x','position_front_y'}};
        RT_veh_raw_trajectory = raw_trajectory(indices_raw_path,:);

        % store lane number indication to check for LC
        RT_veh_lanes = RT_veh_raw_trajectory{:,'lane_number'};

        % Calculate the reference trajectory stations
        RT_veh_diff_station = sqrt(sum(diff(RT_veh_path).^2,2));
        if 0 == RT_veh_diff_station(1)
            RT_veh_station = cumsum([0; RT_veh_diff_station(2:end)]);
            RT_veh_diff_station    = [RT_veh_diff_station(2:end); RT_veh_diff_station(end)];
            RT_veh_raw_trajectory  = RT_veh_raw_trajectory(2:end,:);
            RT_veh_path            = RT_veh_raw_trajectory{:,...
                {'position_front_x','position_front_y'}};
        else
            RT_veh_station = cumsum([0; RT_veh_diff_station]);
            RT_veh_diff_station    = [RT_veh_diff_station; RT_veh_diff_station(end)];  %#ok<AGROW>
        end

        % Redecimate the LCL at a station interval that will give you the
        %   same number of elements as the reference trajectory stations
        station_interval = RT_LCL_lane1.Station(end)/length(RT_veh_station);
        RT_LCL_lane1_new_stations = fcn_calculateNewStations(RT_LCL_lane1,station_interval);
        RT_LCL_lane2_new_stations = fcn_calculateNewStations(RT_LCL_lane2,station_interval);
        RT_LCL_lane3_new_stations = fcn_calculateNewStations(RT_LCL_lane3,station_interval);
        RT_LCL_lane4_new_stations = fcn_calculateNewStations(RT_LCL_lane4,station_interval);

        % Check to see if the station total of the RT is roughly equal to
        %   the station total of the LCLs
        station_difference = RT_LCL_lane1.Station(end) - RT_veh_station(end);
        % If the station difference is too small, set a flag to recalculate
        %   it (later in the code)
        if 2 < station_difference
            recalculate_lane_length = true;
        else
            recalculate_lane_length = false;
        end

        % Check if a lane change occured
        isLanechange = find(all(~diff(RT_veh_lanes))); % if empty, LC happened
        if isempty(isLanechange) == 1
            % a lane change occurred
            disp('Running LC vehicle:')
            disp(index_vehicle)

            % save the index where the lane change occured
            index_lane_change = find(diff(RT_veh_lanes));

            % check if multiple lane changes occurred
            N_laneChanges = length(index_lane_change);

            for i = 1:N_laneChanges % loop through all lane changes that occur   

                % Check to see where the lane change occurred
                % if a lane change occurs 20m from the start or the end of 
                % a path, don't simulate the trajectory
                dist_to_path_start = RT_LCL_lane1_new_stations.Station(index_lane_change(i));
                dist_to_path_end = RT_LCL_lane1_new_stations.Station(end) ...
                    - RT_LCL_lane1_new_stations.Station(index_lane_change(i));

                if 20 < dist_to_path_start && 30 < dist_to_path_end
                    stop_traj_sim = false;

                    % create the index for where to start and end the LC on the
                    %   path (LC discontinuity should be in the middle)
                    lane_change_start_station = RT_veh_station(index_lane_change(i))-0.5*sf;
                    lane_change_end_station = RT_veh_station(index_lane_change(i))+0.5*sf;

                    % calculate station values at which the LC occurs
                    lane_change_stations = (lane_change_start_station:station_interval:lane_change_end_station)';

                    % determine the lane the vehicle starts and ends in
                    lane_start = RT_veh_lanes(index_lane_change(i) - 1);
                    lane_end = RT_veh_lanes(index_lane_change(i) + 1);

                    % retrieve the start and end lane traversal structures
                    [start_lane, end_lane] = fcn_findLaneTraversal(lane_start,lane_end,...
                        RT_LCL_lane1_new_stations,RT_LCL_lane2_new_stations,...
                        RT_LCL_lane3_new_stations,RT_LCL_lane4_new_stations);

                    if recalculate_lane_length
                        % find where X and Y of the lane = X and Y of the first vehicle path
                        %   point
                        RT_veh_path_point_first = RT_veh_path(1,[1,2]);
                        [~,~,~,first_index_path_point,~,~] = ...
                            fcn_Path_snapPointOntoNearestTraversal(RT_veh_path_point_first, start_lane);

                        % Use the found index to parse out what variables
                        % to use
                        % Must parse out X, Y, Z, and station
                        start_lane.X = start_lane.X(first_index_path_point:end);
                        start_lane.Y = start_lane.Y(first_index_path_point:end);
                        start_lane.Z = start_lane.Z(first_index_path_point:end);
                        start_lane.Station = start_lane.Station(first_index_path_point:end);
                        start_lane.Station = start_lane.Station-start_lane.Station(1);
                        end_lane.X   = end_lane.X(first_index_path_point:end);
                        end_lane.Y   = end_lane.Y(first_index_path_point:end);
                        end_lane.Z = end_lane.Z(first_index_path_point:end);
                        end_lane.Station = end_lane.Station(first_index_path_point:end);
                        end_lane.Station = end_lane.Station-end_lane.Station(1);

                        start_lane_end = start_lane.Station(end);
                        end_lane_end = end_lane.Station(end);

                        if start_lane_end <= end_lane_end
                            RT_LCL_end_station = start_lane_end;
                        else
                            RT_LCL_end_station = end_lane_end;
                        end
                        
                        % Redecimate the LCL at a station interval that will give you the
                        %   same number of elements as the reference trajectory stations                       
                        station_interval = RT_LCL_end_station/length(RT_veh_station);
                        start_lane = fcn_calculateNewStations(start_lane,station_interval);
                        end_lane   = fcn_calculateNewStations(end_lane,station_interval);                        
                    end % NOTE: END IF statement to recalculate length

                    % find the indices for before the LC happens
                    indices_LC_before = RT_veh_station<lane_change_stations(1);

                    % find the indices for during the LC
                    indices_LC = RT_veh_station>=lane_change_stations(1) ...
                        & RT_veh_station<=lane_change_stations(end);

                    % find the indices for after the LC happens
                    indices_after_LC = RT_veh_station>lane_change_stations(end);

                    % station input for the lane change smooth function
                    lane_change_stations = RT_veh_station(indices_LC);

                    % smooth the lane change
                    RT_veh_path = fcn_calculateSmoothLaneChange(a,b,c,d,...
                        lane_change_stations,start_lane,end_lane,RT_veh_path,...
                        indices_LC,indices_LC_before,indices_after_LC);

                    % save the smoothed coordinates to the raw trajectory
                    RT_veh_raw_trajectory{:,{'position_front_x'}} = RT_veh_path(:,1);
                    RT_veh_raw_trajectory{:,{'position_front_y'}} = RT_veh_path(:,2);
                else
                    stop_traj_sim = true;
                end % NOTE: END IF statement for dist to path start and end

                %% calculate yaw of the path
                RT_veh_yaw = fcn_Path_calcYawFromPathSegments(RT_veh_path);
                RT_veh_yaw = [RT_veh_yaw; RT_veh_yaw(end)];
            end % NOTE: END FOR loop for number of lane changes
        else
            % a lane change did not occur
            stop_traj_sim = false;

            % When no LC occurs, calculate yaw based on the LCL of travel
            disp('Running no LC vehicle:')
            disp(index_vehicle)

            % determine what lane the vehicle is driving on
            lane_number = RT_veh_lanes(1);

            % snap the RT onto the LCL and calculate yaw
            RT_veh_path_length = length(RT_veh_path);
            RT_veh_yaw = NaN(RT_veh_path_length,1);
            for index_vehicle_path_point = 1:RT_veh_path_length
                % figure out what lane corresponds to the index point
                %   thats the traversal input
                RT_veh_path_point = RT_veh_path(index_vehicle_path_point,[1,2]);
                if lane_number == 1
                    traversal = RT_LCL_lane1;
                elseif lane_number == 2
                    traversal = RT_LCL_lane2;
                elseif lane_number == 3
                    traversal = RT_LCL_lane3;
                elseif lane_number == 4
                    traversal = RT_LCL_lane4;
                end

                [~,~,path_point_yaw,~,~,~] = ...
                    fcn_Path_snapPointOntoNearestTraversal(RT_veh_path_point, traversal);

                RT_veh_yaw(index_vehicle_path_point) = path_point_yaw;
            end
        end % NOTE: END IF statement to check if a lane change occurs

        %% Calcuate the start stop trajectory (if any)
        if stop_traj_sim == false

            % Indices where the vehicle stopped
            temp_var         = (1:size(RT_veh_path,1))';
            % Indices at which the vehicle is at rest
            indices_to_rest  = temp_var(RT_veh_diff_station==0);
            % Indices at which the vehicle begins to stop
            % so indices to start becomes 1, indeces to stop becomes last
            % index (size of trajectory)
            if length(indices_to_rest) < 1
                indices_to_stop = length(RT_veh_path(:,1));
                indices_to_start = 1;
            else
                indices_to_stop  = indices_to_rest([true; 1~=diff(indices_to_rest)]);
                % Indices at which the vehicle begins to move
                indices_to_start = [1; indices_to_rest([1~=diff(indices_to_rest); false])+1];
                % Total number of times a vehicle is stoping
            end
            number_of_stops  = numel(indices_to_stop);
            min_trip_size = 1;
            %% Process a vehicle trajectory between a start and stop
            for index_stop = 1%:number_of_stops

                RT_veh_start_stop_trajectory = RT_veh_raw_trajectory(indices_to_start(index_stop):...
                    indices_to_stop(index_stop),:);

                % Initial conditions
                global_acceleration = zeros(7,1); % global indicates that it's a global variable
                input_states = [RT_veh_start_stop_trajectory.current_speed(1); 0; 0; ...
                    RT_veh_start_stop_trajectory.position_front_x(1); ...
                    RT_veh_start_stop_trajectory.position_front_y(1); ...
                    RT_veh_yaw(indices_to_start(index_stop))];
                U = input_states(1);

                % Reference traversal for the vehicle to track
                RT_veh_sim.X   = RT_veh_start_stop_trajectory.position_front_x;
                RT_veh_sim.Y   = RT_veh_start_stop_trajectory.position_front_y;
                RT_veh_sim.Yaw = RT_veh_yaw(indices_to_start:...
                    indices_to_stop);
                RT_veh_sim.Station  = RT_veh_station(indices_to_start:...
                    indices_to_stop);
                RT_veh_sim.Velocity = RT_veh_start_stop_trajectory.current_speed;

                % Time parameters
                % Note: TotalTime is the time taken in Aimsun, could add a slack to this
                total_time_matrix = RT_veh_raw_trajectory.aimsun_time;
                total_time = RT_veh_raw_trajectory.aimsun_time(indices_to_stop(index_stop))-...
                    RT_veh_raw_trajectory.aimsun_time(indices_to_start(index_stop));
                
                % check to make sure time doesn't increase by more than 0.1s
                total_time_check_index = diff(total_time_matrix);
                total_time_check = find(diff(total_time_check_index) > 0.1);
                % Duration where vehicle is at rest
                if index_stop ~= index_stop
                    duration_of_rest = RT_veh.aimsun_time(indices_to_stop(index_stop))-...
                        RT_veh.aimsun_time(indices_to_start(index_stop+1));
                else
                    duration_of_rest = 0;
                end

                % Define variable to store veh sim information
                t_vector = 0:deltaT:total_time;
                N_timesteps = length(t_vector); % calculate # of time steps sim needs to run
                matlab_time   = NaN(N_timesteps,1);
                matlab_States = NaN(N_timesteps,9);
                matlab_pose   = NaN(N_timesteps,3);

                tire_forces_fl_sq = NaN(N_timesteps,1);
                tire_forces_fr_sq = NaN(N_timesteps,1);
                tire_forces_rl_sq = NaN(N_timesteps,1);
                tire_forces_rr_sq = NaN(N_timesteps,1);

                normal_force_fl = NaN(N_timesteps,1);
                normal_force_fr = NaN(N_timesteps,1);
                normal_force_rl = NaN(N_timesteps,1);
                normal_force_rr = NaN(N_timesteps,1);

                for ith_time = 1:N_timesteps
                    t = t_vector(ith_time);
                    matlab_time(ith_time)       = t;
                    matlab_States(ith_time,1:3) = input_states(1:3)';
                    matlab_pose(ith_time,:)     = input_states(4:6)';

                    %% Controller: Steering + Velocity
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Note: Controller need to be tuned, particularly for the
                    % velocity
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % add if statement with total time to set target_U = 0
                    pose = matlab_pose(ith_time,:)';
                    [target_lookAhead_pose,target_U] = ...
                        fcn_VD_snapLookAheadPoseOnToTraversal(pose,RT_veh_sim,controller);
                    steering_angle = fcn_VD_lookAheadLatController(pose,target_lookAhead_pose,...
                        controller);
                    if 0<=U
                        wheel_slip = fcn_VD_velocityControllerSlipBasis(U,target_U,controller);
                    else
                        wheel_slip = zeros(4,1);
                    end

                    %% 3-DoF Vehicle Model
                    flag_update = true; % set it to to true before every call to RK4 method
                    if 0.5<=U
                        [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dt3dofModelForController(t,y,...
                            steering_angle,wheel_slip,...
                            vehicle,road_properties,friction_coefficient,type_of_transfer),...
                            input_states,t,deltaT);
                        U = y(1); V = y(2); r = y(3);
                        [~,normal_force, tire_forces] = fcn_VD_dt3dofModelForController(t,y,...
                            steering_angle,wheel_slip,...
                            vehicle,road_properties,friction_coefficient,type_of_transfer);
                        input_states = y; clear y;
                    elseif 0<=U
                        kinematic_input_states = [input_states(1); input_states(4:6)];
                        [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dtKinematicModelForController(t,y,...
                            steering_angle,wheel_slip*30000*vehicle.Re,...
                            vehicle,road_properties,type_of_transfer),...
                            kinematic_input_states,t,deltaT);
                        U = y(1); V = 0; %omega = (U/vehicle.Re)*ones(4,1);
                        r = fcn_VD_kinematicYawRate(U,steering_angle,vehicle);
                        input_states = [U; V; r; y(2:4)]; clear y;
                    end

                    % store tire forces squared
                    tire_forces_fl_sq(ith_time) = tire_forces(1,1)^2 + tire_forces(1,2)^2;
                    tire_forces_fr_sq(ith_time) = tire_forces(2,1)^2 + tire_forces(2,2)^2;
                    tire_forces_rl_sq(ith_time) = tire_forces(3,1)^2 + tire_forces(3,2)^2;
                    tire_forces_rr_sq(ith_time) = tire_forces(4,1)^2 + tire_forces(4,2)^2;

                    % store normal force
                    normal_force_fl(ith_time) = normal_force(1,1);
                    normal_force_fr(ith_time) = normal_force(2,1);
                    normal_force_rl(ith_time) = normal_force(3,1);
                    normal_force_rr(ith_time) = normal_force(4,1);

                    matlab_States(ith_time,8:9) = global_acceleration(1:2)';
                end % NOTE: END FOR loop for vehicle controller and model

                normal_forces = [normal_force_fl normal_force_fr...
                    normal_force_rl normal_force_rr];
                tire_forces_sq = [tire_forces_fl_sq tire_forces_fr_sq...
                    tire_forces_rl_sq tire_forces_rr_sq];

                %% Snap and interpolate vehicle data to the road

                VT_veh_path = matlab_pose(:,[1,2]);
                VT_veh_traversal = fcn_Path_convertPathToTraversalStructure(VT_veh_path);

                % cut off first 20 m of data to account for non-matching
                %   initial conditions
                indices_rows_to_select_veh = find(VT_veh_traversal.Station >= cut_off_length);
                VT_veh_path = VT_veh_path(indices_rows_to_select_veh,:);
                VT_veh_traversal.Station = VT_veh_traversal.Station(indices_rows_to_select_veh);

                friction_demand_fl = sqrt(tire_forces_fl_sq)./normal_force_fl;
                friction_demand_fl = friction_demand_fl(indices_rows_to_select_veh);

                friction_demand_fr = sqrt(tire_forces_fr_sq)./normal_force_fr;
                friction_demand_fr = friction_demand_fr(indices_rows_to_select_veh);

                friction_demand_rl = sqrt(tire_forces_rl_sq)./normal_force_rl;
                friction_demand_rl = friction_demand_rl(indices_rows_to_select_veh);

                friction_demand_rr = sqrt(tire_forces_rr_sq)./normal_force_rr;
                friction_demand_rr = friction_demand_rr(indices_rows_to_select_veh);

                % Longitudinal and lateral acceleration
                lon_accel = matlab_States(:,8);
                lon_accel = lon_accel(indices_rows_to_select_veh);
                lat_accel = matlab_States(:,9);
                lat_accel = lat_accel(indices_rows_to_select_veh);

                % Longitudinal and lateral velocity
                lon_vel = matlab_States(:,1);
                lon_vel = lon_vel(indices_rows_to_select_veh);
                % lat_vel = matlab_States(:,2);
                % lat_vel = lat_vel(rows_to_select_veh);

                % Yaw rate
                % yaw_rate = matlab_States(:,3);
                % yaw_rate = yaw_rate(rows_to_select_veh);
            end % NOTE: FOR loop for number of stops
            %% Plot the results
            if flag.plot
                cg_station = VT_veh_traversal.Station;
                fcn_VD_plotTrajectory([VT_veh_traversal.X VT_veh_traversal.Y],01); % Plot output trajectory
                fcn_VD_plotStationLongitudinalAcceleration(cg_station,lon_accel,02); % Plot longitudinal acceleration
                fcn_VD_plotStationLateralAcceleration(cg_station,lat_accel,03); % Plot lateral acceleration
                fcn_VD_plotStationLongitudinalVelocity(cg_station,lon_vel,04); % Plot longitudinal velocity
                fcn_VD_plotStationFrictionDemand(cg_station,friction_demand_fl,05);
                fcn_VD_plotStationFrictionDemand(cg_station,friction_demand_fr,06); % Plot force ratio
                fcn_VD_plotStationFrictionDemand(cg_station,friction_demand_rl,07); % Plot force ratio
                fcn_VD_plotStationFrictionDemand(cg_station,friction_demand_rr,08); % Plot force ratio
            end
        end
    end
end % NOTE: END FOR loop for evaluating section ID trajectories