function [vs, ws, stop] = controlParkingLim(q, goalPose, goalId, vehicleId, collision, inside, map)
% Control to parking area with limited angle gamma
%
%Inputs
% q:         Current vehicle configuration [x; y; phi; gamma]
% goalPose:  Goal pose [x_goal; y_goal; phi_goal]
% goalId:    ID of the goal parking spot
% vehicleId: Vehicle ID
% collision: Has the vehicle collided with an obstacle or another vehicle?
% inside:    Is the vehicle inside the goal parking area?
% map:       Map with data about the environment
%
%Outputs
% vs:   Linear velocity (in m/s)
% ws:   Angular velocity of the front part (in rad/s)
% stop: Should the control stop? Emit if the goal is reached.

%NOTE To store some values between function calls, use persistent variables
% persistent a % Declaration of persistent variable
% if isempty(a), a = 0; end % Initialization of the persistent variable

%EXAMPLES
% map.parkingPoses(:,goalId) % Goal pose ([x; y; phi]), same as goalPose function argument
% map.parkingAreas{goalId} % Outline of the goal area
% 
% map.obstacles % Cell array of the outlines of all static obstacles
% map.vehicleObstacles % Cell array of the outlines of all vehicles that are considered as obstacles (without the current vehicle)
% map.vehiclePoses % Poses of all vehicles (pose [x; y; phi] in each column) that are considered as obstacles (without the current vehicle)
% map.parkingAreas % Cell array of the outlines of all parking areas
% map.parkingPoses % Poses of all parking spots (pose [x; y; phi] in each column)
% 
% map.isCollision(q(1:3)) % Check if the vehicle at the given pose is in collision
% map.isInside(q(1:3), map.parkingAreas{goalId}) % Check if the vehicle is inside the given area
%
% See Map.m for more.

%TODO Write your code here ...
vs = 0;
ws = 0;
stop = true;