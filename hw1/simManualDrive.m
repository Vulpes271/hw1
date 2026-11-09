function simManualDrive(q0, goalIds, gammaMax, constraints)
%Simulation of the vehicle using manual drive with the keyboard
%
%Inputs
% q0:          Robot initial configuration [x; y; phi; gamma] or empty (default) for a random configuration in the starting area
% goalIds:     Array of goal parking IDs (0 for random goal parking ID)
% gammaMax:    Maximum gamma (default is pi, i.e. unlimited)
% constraints: Vehicle constraints object (default is with velocity constraints)
if nargin < 1
    q0 = [];
end
if nargin < 2
    goalIds = 11;
end
if nargin < 3 || isempty(gammaMax)
    gammaMax = pi;
end
if nargin < 4 || isempty(constraints)
    constraints = VehicleConstraints(0.3, 1);
end

clear('manualDrive'); % Clear persistent values
constraints.reset();

q0 = q0(:);

goalIds(12:end) = [];
goalIds(goalIds<0) = 0;
goalIds(goalIds>11) = 0;
ava = setdiff(1:11, goalIds);
ava = ava(randperm(length(ava)));
n = sum(goalIds==0);
goalIds(goalIds==0) = ava(1:n);
N = length(goalIds);
if N == 0
    error('No goals.');
end
if N ~= length(unique(goalIds))
    error('Goal IDs must be unique.');
end

chrono = Chrono();
chrono.T = 0.02; % Sampling time, in s
chrono.tSim = inf; % Simulation time, in s
chrono.realtime = true;

map = Map();

anim = VehicleAnimation(chrono, map);

vehicleId = 1;

qStart = map.getVehicleInStartingArea(q0, gammaMax);
q = qStart;
goalId = goalIds(vehicleId);
anim.spawnVehicle(q, goalId);

go = [];
collision = false;
goalArea = map.parkingAreas{goalId};
inside = false;
stop = false;
qLast = q;

chrono.init();
while chrono.loop() % Simulation loop
    if ~stop
        [vs, ws, stop] = manualDrive(anim.key, anim.count); % Drive the vehicle with the keyboard
    end
    if stop
        vs = 0;
        ws = 0;
    end
    
    [vs, ws] = constraints.update(vs, ws, chrono.T); % Apply constraints

    anim.update(q, vs, ws, collision, inside); % Update animation

    if stop && all(abs(q - qLast) < 1e-6)
        go = [go, q(1:3)];
        
        vehicleId = vehicleId + 1;
        if vehicleId <= N
            map.setVehiclesAsObstacles(go);
            constraints.reset();
            qStart = map.getVehicleInStartingArea(q0, gammaMax);
            q = qStart;
            goalId = goalIds(vehicleId);
            anim.spawnVehicle(q, goalId);
            chrono.reset();
            collision = false;
            goalArea = map.parkingAreas{goalId};
            inside = false;
            stop = false;
            qLast = q;
        else
            break;
        end
    else
        qLast = q;
        q1 = q;
        q = updateVehicle(q, vs, ws, chrono.T); % Simulate vehicle
        q(4) = min(max(-gammaMax, wrapToPi(q(4))), gammaMax);
    
        collision = map.isCollision(q);
        if collision
            q = q1;
        end

        inside = map.isInside(q, goalArea);
    end
end
anim.finish();