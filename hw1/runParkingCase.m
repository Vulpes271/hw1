function data = runParkingCase(q0, goals, limited, noise)
% Numericni preizkus brez graficnih posodobitev pri vsakem vzorcu.
% Uporablja izvirni updateVehicle (ode45), Map in VehicleConstraints.
if nargin < 4, noise = 1; end
map = Map();
gammaMax = pi;
controller = @controlParking;
if limited
    gammaMax = pi/6;
    controller = @controlParkingLim;
end
available = setdiff(1:11,goals);
available = available(randperm(numel(available)));
goals(goals == 0) = available(1:nnz(goals == 0));
parked = [];
data = struct([]);
for vehicleId = 1:numel(goals)
    clear controlParking controlParkingLim
    constraints = VehicleConstraints(0.3,1);
    goalId = goals(vehicleId);
    goal = map.parkingPoses(:,goalId);
    area = map.parkingAreas{goalId};
    qStart = map.getVehicleInStartingArea(q0,gammaMax);
    q = qStart;
    qLast = q;
    stopped = false;
    collision = false;
    inside = false;
    poses = nan(4,12001);
    inputs = nan(2,12001);
    collisions = false(1,12001);
    for k = 1:12001
        measured = q+noise*randn(4,1).*[.001;.001;.002;.001];
        if ~stopped
            [vs,ws,stopped] = controller(measured,goal,goalId, ...
                vehicleId,collision,inside,map);
        end
        if stopped, vs = 0; ws = 0; end
        [vs,ws] = constraints.update(vs,ws,0.01);
        poses(:,k) = q;
        inputs(:,k) = [vs;ws];
        collisions(k) = collision;
        if stopped && all(abs(q-qLast) < 1e-6), break; end
        qLast = q;
        next = updateVehicle(q,vs,ws,0.01);
        if limited
            next(4) = max(-gammaMax,min(gammaMax,next(4)));
        end
        collision = collides(next,map);
        if ~collision, q = next; end
        inside = map.isInside(q,area);
    end
    if ~stopped
        error('runParkingCase:Timeout','P%d: prekoracenih 120 s.',goalId);
    end
    result = struct('q0',q0,'qStart',qStart, ...
        'vehicleId',vehicleId,'goalId',goalId,'goalPose',goal, ...
        'qEnd',q,'tEnd',(k-1)*.01,'inside',inside,'noise',noise, ...
        't',(0:k-1)*.01,'vs',inputs(1,1:k),'ws',inputs(2,1:k), ...
        'vl',inputs(1,1:k)-.043*inputs(2,1:k)/2, ...
        'vr',inputs(1,1:k)+.043*inputs(2,1:k)/2, ...
        'c',collisions(1:k),'x',poses(1,1:k),'y',poses(2,1:k), ...
        'phi',poses(3,1:k),'gamma',poses(4,1:k));
    assert(inside && ~map.isCollision(q),'Neveljavna koncna lega.');
    assert(norm(q(1:2)-goal(1:2)) < .001,'Prevelika napaka polozaja.');
    assert(abs(wrapToPi(q(3)-goal(3))) < pi/360, ...
        'Prevelika napaka orientacije.');
    assert(max(abs([result.vl,result.vr])) <= .3+1e-10);
    acceleration = diff([0,result.vl;0,result.vr],1,2)/.01;
    assert(max(abs(acceleration),[],'all') <= 1+1e-10);
    if limited, assert(max(abs(result.gamma)) <= pi/6+1e-10); end
    data = [data,result]; %#ok<AGROW>
    parked = [parked,q(1:3)]; %#ok<AGROW>
    map.setVehiclesAsObstacles(parked);
    fprintf('P%02d: t=%5.2f s, ed=%.3f mm, ephi=%.3f deg, trki=%d\n', ...
        goalId,result.tEnd,1000*norm(q(1:2)-goal(1:2)), ...
        rad2deg(abs(wrapToPi(q(3)-goal(3)))),nnz(result.c));
end
end

function collision = collides(q,map)
% SAT je ekvivalenten preverjanju presekov pravokotnikov v Map.
% Izognemo se izdelavi objektov polyshape pri vsakem vzorcu.
p = map.getOutline(q);
p = p(:,1:4);
u = [cos(q(3));sin(q(3))];
v = [-u(2);u(1)];
collision = false;
for polygon = [map.obstacles,map.vehicleObstacles]
    b = polygon{1}(:,1:4);
    a = b(:,2)-b(:,1);
    a = a/norm(a);
    axes = [u,v,a,[-a(2);a(1)]];
    separate = false;
    for axis = axes
        first = axis.'*p;
        second = axis.'*b;
        if max(first) <= min(second) || max(second) <= min(first)
            separate = true;
            break
        end
    end
    if ~separate, collision = true; return; end
end
end
