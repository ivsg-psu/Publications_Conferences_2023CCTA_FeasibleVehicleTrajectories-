%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% Define inputs and parameters
global flag_update global_acceleration

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
vehicle.friction_ratio = 1; % [No units]

controller.look_ahead_distance = 20; % look-ahead distance [meters]
controller.steering_Pgain      = 0.1; % P gain for steering control
controller.velocity_Pgain      = 200; % P gain for steering control

% Parameters and initial conditions for simulink and matlab model
road_properties.grade = 0; road_properties.bank_angle = 0; % road properties
friction_coefficient  = 0.9*ones(4,1);

type_of_transfer = 'default';

TotalTime = 10; % [seconds]
deltaT    = 0.01; % [seconds]
number_of_samples = floor(10/deltaT)+1;

%%
matlab_time = NaN(number_of_samples,1);
matlab_States = NaN(number_of_samples,1);
matlab_pose   = NaN(number_of_samples,3);

global_acceleration = zeros(7,1);
input_states = [20; 0; 0; 0]; % initial conditions
counter = 1;
for t = 0:deltaT:TotalTime
    matlab_time(counter) = t;
    matlab_States(counter,1) = input_states(1);
    matlab_pose(counter,:)   = input_states(2:4)';
    
    steering_angle = [2; 2; 0; 0]*(pi/180);
    wheel_torque   = [0; 0; 100; 100];

    %% 7-DoF Vehicle Model
    flag_update = true; % set it to to true before every call to RK4 method
    [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dtKinematicModelForController(t,y,...
        steering_angle,wheel_torque,...
        vehicle,road_properties,type_of_transfer),...
        input_states,t,deltaT);
    input_states = y; clear y;
    
    matlab_States(counter,2:3) = global_acceleration(1:2)';
    counter = counter+1;
end

%% Plot the trajectory
figure(12345)
clf
plot(matlab_pose(:,1),matlab_pose(:,2))
grid on
axis equal
