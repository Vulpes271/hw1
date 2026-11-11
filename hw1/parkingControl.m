function [vs, ws, stop, state] = parkingControl(q, goal, ...
    collision, inside, map, gammaMax, state)
% Povratno vodenje po odsekih; pretvorba z inverzno kinematiko.
% AMS Control: sledenje poti (10-11), krmiljenje smeri/polozaja (15-17).
D = 0.1207;
vs = 0;
ws = 0;
stop = false;
if ~isfield(state,'path')
    state.path = parkingPlan(q,goal,map,gammaMax);
    state.index = 1;
    state.q = q;
    state.phase = 1;
    state.stable = 0;
    state.constraints = VehicleConstraints(0.3,1);
    state.u = [0;0];
    state.replans = 0;
    state.idle = 0;
end
% Napoved s kinematiko odpravi zamik filtra med voznjo.
a = derivKinematics(0,state.q,state.u(1),state.u(2),D);
b = derivKinematics(0,state.q+0.005*a,state.u(1),state.u(2),D);
state.q = state.q+0.01*b;
if gammaMax < pi/2
    state.q(4) = max(-gammaMax,min(gammaMax,state.q(4)));
end
delta = q-state.q;
delta(3:4) = wrapToPi(delta(3:4));
gain = 0.08;
if norm(goal(1:2)-state.q(1:2)) < 0.01, gain = 0.03; end
state.q = state.q+gain*delta;
q = state.q;
path = state.path;
i = state.index;
N = size(path,2);

% Na prehodu med premico in lokom najprej nastavimo krmilni kot.
direction = path(5,i);
last = i;
while last < N && path(5,last+1) == direction && ...
        abs(path(4,last+1)-path(4,i)) < 1e-6
    last = last+1;
end
if direction == 0
    errorPhi = wrapToPi(path(3,last)-q(3));
    gamma = pi/2;
    if state.phase == 1
        if abs(wrapToPi(gamma-q(4))) < 0.015, state.phase = 2; end
    else
        vs = max(-0.09,min(0.09,1.5*D*errorPhi));
        if abs(errorPhi) < 0.003
            vs = 0;
            state.index = min(last+1,N);
            state.phase = 1;
        end
    end
else
    % Najblizja lega na trenutnem odseku.
    candidates = i:min(last,i+25);
    [~,j] = min(sum((path(1:2,candidates)-q(1:2)).^2,1));
    i = candidates(j);
    state.index = i;
    distance = norm(path(1:2,last)-q(1:2));
    tangent = [cos(path(3,last)),sin(path(3,last))];
    remaining = direction*tangent*(path(1:2,last)-q(1:2));
    if last < N && distance < 0.015 && remaining < 0.0004
        state.index = last+1;
        state.phase = 1;
        [state.u(1),state.u(2)] = state.constraints.update(0,0,0.01);
        return
    end
    if last == N && (distance < 0.0015 || ...
            (distance < 0.015 && remaining < 0.0005))
        % Koncno vodenje po smeri parkirnega mesta, brez atan2(0,0).
        offset = goal(1:2)-q(1:2);
        longitudinal = [cos(goal(3)),sin(goal(3))]*offset;
        lateral = [-sin(goal(3)),cos(goal(3))]*offset;
        phi = goal(3)+direction*atan2(lateral,0.10);
        v = max(-0.015,min(0.015,0.8*longitudinal));
        omega = 2*wrapToPi(phi-q(3));
        gamma = atan(D*omega/max(abs(v),0.008))*direction;
    else
        % Projekcija na odsek: referencna lega in orientacija.
        j = min(i+1,last);
        segment = path(1:2,j)-path(1:2,i);
        fraction = segment.'*(q(1:2)-path(1:2,i));
        fraction = max(0,min(1,fraction/max(sum(segment.^2),1e-10)));
        target = path(1:2,i)+fraction*segment;
        phi = path(3,i)+fraction*wrapToPi(path(3,j)-path(3,i));
        lateral = [-sin(q(3)),cos(q(3))]*(target-q(1:2));
        minimum = 0.02;
        if last == N, minimum = 0.004; end
        v = direction*min(0.10,max(minimum,0.8*distance));
        omega = v*path(4,j)+50*abs(v)*wrapToPi(phi-q(3)) ...
            +1250*v*lateral;
        gamma = atan(D*omega/v);
    end
    gamma = max(-gammaMax,min(gammaMax,gamma));
    if state.phase == 1
        if abs(wrapToPi(gamma-q(4))) < 0.025, state.phase = 2; end
    else
        vs = v/max(0.3,cos(q(4)));
    end
end
ws = vs*sin(q(4))/D+10*wrapToPi(gamma-q(4));
positionError = norm(goal(1:2)-q(1:2));
angleError = abs(wrapToPi(goal(3)-q(3)));
% Brez omejitve gamma lahko zadnji majhen popravek smeri opravimo na mestu.
if gammaMax == pi/2 && direction ~= 0 && last == N && ...
        abs(state.u(1)) < 0.002 && ...
        positionError < 0.0007 && angleError >= 0.0075
    state.path = [q(1:2);goal(3);0;0];
    state.index = 1;
    state.phase = 1;
    vs = 0;
    ws = 0;
end
if inside && positionError < 0.0007 && angleError < 0.0075
    state.stable = state.stable+1;
    vs = 0;
    ws = 0;
    stop = state.stable >= 25;
else
    state.stable = 0;
end
% Ob vecjem preostanku ali trku ponovno izberemo vmesne lege.
longitudinal = [cos(goal(3)),sin(goal(3))]*(goal(1:2)-q(1:2));
idle = last == N && abs(state.u(1)) < 0.002 && ...
    ((direction == 0 && positionError > 0.0007) || ...
    (direction ~= 0 && abs(longitudinal) < 0.0002 && ...
    (positionError > 0.00095 || angleError > 0.0087)));
if idle, state.idle = state.idle+1; else, state.idle = 0; end
retry = state.idle >= 100;
if (collision || retry) && state.replans < 3
    state.path = parkingPlan(q,goal,map,gammaMax);
    state.index = 1;
    state.phase = 1;
    state.replans = state.replans+1;
    state.idle = 0;
    vs = 0;
    ws = 0;
end
[state.u(1),state.u(2)] = state.constraints.update(vs,ws,0.01);
end
