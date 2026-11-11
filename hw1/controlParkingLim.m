function [vs, ws, stop] = controlParkingLim(q, goalPose, ~, ...
    ~, collision, inside, map)
% Naloga 3: isti regulator, z omejitvijo zasuka prednjega dela.
persistent state
if isempty(state), state = struct(); end
[vs, ws, stop, state] = parkingControl(q, goalPose, ...
    collision, inside, map, pi/6, state);
end
