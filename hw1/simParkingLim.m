function data = simParkingLim(q0, goalIds, noise, animate, constraints)
%Simulation of control to parking area with limited angle gamma
%
%Inputs
% q0:          Robot initial configuration [x; y; phi; gamma] or empty (default) for a random configuration in the starting area
% goalIds:     Array of goal parking IDs (0 for random goal parking ID)
% noise:       Enable noise (1 - enabled (default), 0 - disabled)
% animate:     Animation decimation (0 for no animation, default is 1)
% constraints: Vehicle constraints object (default is with velocity constraints)
%
%Outputs
% data: Structure array with collected data during the simulation
if nargin < 1
    q0 = [];
end
if nargin < 2
    goalIds = 11;
end
if nargin < 3 || isempty(noise)
    noise = 1;
end
if nargin < 4 || isempty(animate)
    animate = 1;
end
if nargin < 5 || isempty(constraints)
    constraints = VehicleConstraints(0.3, 1);
end

clear('controlParkingLim'); % Clear persistent values
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
chrono.T = 0.01; % Sampling time, in s
chrono.tSim = 120; % Simulation time, in s

gammaMax = 30/180*pi;

map = Map();

anim = VehicleAnimation(chrono, map);
anim.decimation = animate;

vehicleId = 1;

qStart = map.getVehicleInStartingArea(q0, gammaMax);
q = qStart;
goalId = goalIds(vehicleId);
anim.spawnVehicle(q, goalId);

io = [];
eo = [];
go = [];
to = [];
collision = false;
goalPose = map.parkingPoses(:,goalId);
goalArea = map.parkingAreas{goalId};
inside = false;
stop = false;
qLast = q;
data = [];

chrono.init();
while chrono.loop() % Simulation loop
    qn = q + randn(4,1).*[0.001; 0.001; 0.002; 0.001]*noise;
    if ~stop
        [vs, ws, stop] = controlParkingLim(qn, goalPose, goalId, vehicleId, collision, inside, map); % Control
    end
    if stop
        vs = 0;
        ws = 0;
    end
    
    [vs, ws] = constraints.update(vs, ws, chrono.T); % Apply constraints

    anim.update(q, vs, ws, collision, inside); % Update animation

    if stop && all(abs(q - qLast) < 1e-6)
        eo = [eo, goalPose - q(1:3)];
        io = [io, inside];
        to = [to, chrono.t];
        go = [go, q(1:3)];
        data = [data, anim.collect(q0, qStart, vehicleId, goalId, goalPose, q, chrono.t, inside, noise)];
        
        vehicleId = vehicleId + 1;
        if vehicleId <= N
            map.setVehiclesAsObstacles(go);
            clear('controlParkingLim'); % Clear persistent values
            constraints.reset();
            qStart = map.getVehicleInStartingArea(q0, gammaMax);
            q = qStart;
            goalId = goalIds(vehicleId);
            anim.spawnVehicle(q, goalId);
            chrono.reset();
            collision = false;
            goalPose = map.parkingPoses(:,goalId);
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

fprintf('\n# Summary\n');
for i = 1:length(to)
    fprintf(' - Robot %d: inside=%d, ed=%8.4f m, ephi=%8.4f rad, t=%8.4f s\n', i, io(i), sqrt(eo(1,i)^2 + eo(2,i)^2), wrapToPi(eo(3,i)), to(i));
end
