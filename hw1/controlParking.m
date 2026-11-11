function [vs, ws, stop] = controlParking(q, goalPose, ~, ...
    ~, collision, inside, map)
% Nalogi 1 in 2: skupno vodenje, neodvisno od stevilke parkirisca.
persistent state
if isempty(state), state = struct(); end
[vs, ws, stop, state] = parkingControl(q, goalPose, ...
    collision, inside, map, pi/2, state);
end
