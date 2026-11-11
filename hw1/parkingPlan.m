function path = parkingPlan(start, goal, map, gammaMax)
% Iscemo od parkirisca navzven, nato obrnemo smer voznje.
% Tako najprej resimo ozki del manevra, tudi med dvema voziloma.
path = search(goal,start(1:3),map,gammaMax);
path = path(:,end:-1:1);
path(5,:) = -path(5,:);
path(4:5,2:end) = path(4:5,1:end-1);
path(4:5,1) = path(4:5,2);
end

function path = search(start, goal, map, gammaMax)
% Vmesne lege na premicah in kroznicah (AMS Control, str. 10).
% A* iz AMS Path Planning izbere prehod med ovirami in vozili.
% Vrstice poti: x, y, phi, krivina, smer (+1/-1, 0 za obrat).
D = 0.1207;
% Zmerna referencna ukrivljenost pusti rezervo za povratno vezavo.
radius = D/tan(0.95*min(gammaMax,pi/6));
turns = [-1, 0, 1]/radius;
obstacles = [map.obstacles, map.vehicleObstacles];
rect = zeros(5, numel(obstacles));
for j = 1:numel(obstacles)
    p = obstacles{j}(:,1:4);
    u = p(:,2)-p(:,1);
    rect(:,j) = [mean(p,2); norm(u)/2; ...
        norm(p(:,3)-p(:,2))/2; atan2(u(2),u(1))];
end

% Rozlocimo smer voznje, da kaznujemo nepotrebne menjave smeri.
resolution = 0.02;
nxy = ceil([map.width, map.height]/resolution)+1;
field = gridDistances(rect,goal,nxy,resolution);
nt = 72;
coarseCount = prod(nxy)*nt*2;
fineN = 183;
best = inf(coarseCount+fineN^2*360*2,1);
capacity = 300000;
poses = zeros(3, capacity);
parents = zeros(1, capacity);
actions = zeros(3, capacity);
cost = inf(1, capacity);
poses(:,1) = start(1:3);
cost(1) = 0;
count = 1;
heap = zeros(2, capacity);
heapSize = 0;
push(1, heuristic(start));
found = 0;
tail = [];
expanded = 0;

while heapSize > 0 && count < capacity-8
    id = pop();
    p = poses(:,id);
    key = index(p, actions(2,id));
    if cost(id) > best(key), continue; end
    expanded = expanded+1;
    step = 0.06;
    if norm(p(1:2)-start(1:2)) < 0.30, step = 0.015; end
    if norm(p(1:2)-start(1:2)) < 0.18, step = 0.005; end
    if id == 1, step = 0.01; end

    % Preslikava na natancen cilj s tangentnimi premicami in loki.
    if id > 1 && norm(p(1:2)-goal(1:2)) < 0.55
        tail = connect(p, goal);
        if ~isempty(tail)
            found = id;
            break
        end
    end

    for direction = [-1, 1]
        if id == 1
            for candidateStep = [0.10,0.04,0.01]
                if free(arc(p,direction*candidateStep,0))
                    step = candidateStep;
                    break
                end
            end
        end
        for curvature = turns
            if id == 1 && curvature ~= 0, continue; end
            segment = arc(p, direction*step, curvature);
            if ~free(segment), continue; end
            next = segment(1:3,end);
            key = index(next, direction);
            g = cost(id)+step;
            g = g+0.035*(actions(2,id)*direction < 0);
            g = g+0.003*(curvature ~= 0);
            if g >= best(key), continue; end
            count = count+1;
            poses(:,count) = next;
            parents(count) = id;
            actions(:,count) = [curvature; direction; step];
            cost(count) = g;
            best(key) = g;
            push(count, g+3*heuristic(next));
        end
    end
end
if found == 0
    error('parkingPlan:NoPath', ...
        'Ni poti po %d legah. Zacetek: [%.4f %.4f %.4f].', ...
        expanded,start(1),start(2),start(3));
end
ids = found;
while parents(ids(1)) ~= 0
    ids = [parents(ids(1)), ids]; %#ok<AGROW>
end
path = zeros(5,0);
for j = 2:numel(ids)
    a = actions(:,ids(j));
    p = poses(:,ids(j-1));
    segment = arc(p,a(2)*a(3),a(1));
    path = [path, segment(:,2:end)]; %#ok<AGROW>
end
path = [[start(1:3); pathDirection(path,tail)], path, tail];

    function h = heuristic(p)
        h = norm(p(1:2)-goal(1:2));
        xy = min(max(round(p(1:2)/resolution)+1,1),nxy(:));
        grid = field(xy(1),xy(2));
        if isfinite(grid), h = max(h,grid-resolution*sqrt(2)); end
        h = h+0.08*max(0,1-h/0.3)*abs(wrapToPi(p(3)-goal(3)));
    end

    function key = index(p, direction)
        if norm(p(1:2)-start(1:2)) < 0.18
            xy = round((p(1:2)-start(1:2)+0.18)/0.002)+1;
            angle = mod(round(p(3)*360/(2*pi)),360)+1;
            key = coarseCount+sub2ind([fineN,fineN,360,2], ...
                xy(1),xy(2),angle,1+(direction<0));
            return
        end
        xy = round(p(1:2)/resolution)+1;
        xy = min(max(xy,1),nxy(:));
        angle = mod(round(p(3)*nt/(2*pi)),nt)+1;
        key = sub2ind([nxy,nt,2],xy(1),xy(2),angle,1+(direction<0));
    end

    function ok = free(p)
        % SAT za pravokotnike: ista geometrija kot Map.getOutline.
        % Dodamo 6 mm rezerve za pogreske povratnega vodenja.
        phi = p(3,:);
        u = [cos(phi); sin(phi)];
        v = [-sin(phi); cos(phi)];
        center = p(1:2,:)+(map.H/2-map.I)*u;
        distance = min(sqrt(sum((p(1:2,:)-start(1:2)).^2,1)), ...
            sqrt(sum((p(1:2,:)-goal(1:2)).^2,1)));
        margin = 0.006*min(1,distance/0.04);
        h = map.H/2+margin;
        w = map.W/2+margin;
        ok = true;
        for b = rect
            a = [cos(b(5)); sin(b(5))];
            n = [-a(2); a(1)];
            delta = center-b(1:2);
            ca = abs(a.'*u);
            sa = abs(a.'*v);
            separate = abs(a.'*delta) > b(3)+h.*ca+w.*sa;
            separate = separate | abs(n.'*delta) > b(4)+h.*sa+w.*ca;
            separate = separate | abs(sum(u.*delta,1)) > ...
                h+b(3)*ca+b(4)*sa;
            separate = separate | abs(sum(v.*delta,1)) > ...
                w+b(3)*sa+b(4)*ca;
            if any(~separate), ok = false; return; end
        end
    end

    function result = connect(p, target)
        % Zunanje/notranje skupne tangente dveh kroznic.
        result = [];
        final = target;
        for lead = [0.10,0.04,0.01]
            shortest = inf;
            for travel = [-1, 1]
                target = final-[travel*lead*cos(final(3)); ...
                    travel*lead*sin(final(3));0];
                a0 = p(3)+(travel<0)*pi;
                a1 = target(3)+(travel<0)*pi;
                for s0 = [-1, 1]
                    for s1 = [-1, 1]
                        c0 = p(1:2)+s0*radius*[-sin(a0);cos(a0)];
                        c1 = target(1:2)+s1*radius*[-sin(a1);cos(a1)];
                        delta = c1-c0;
                        distance = norm(delta);
                        offset = (s1-s0)*radius;
                        if distance < abs(offset)+1e-9, continue; end
                        tangent = atan2(delta(2),delta(1))-asin(offset/distance);
                        angles = mod([s0*(tangent-a0),s1*(a1-tangent)],2*pi);
                        angles(angles>2*pi-1e-7) = 0;
                        straight = sqrt(distance^2-offset^2);
                        length = radius*sum(angles)+straight;
                        if length > min(shortest,1.1), continue; end
                        b0 = arc(p,travel*radius*angles(1),travel*s0/radius);
                        b1 = arc(b0(1:3,end),travel*straight,0);
                        b2 = arc(b1(1:3,end),travel*radius*angles(2),travel*s1/radius);
                        b3 = arc(b2(1:3,end),travel*lead,0);
                        candidate = [b0,b1(:,2:end),b2(:,2:end),b3(:,2:end)];
                        candidate(5,:) = travel;
                        if free(candidate)
                            result = candidate;
                            shortest = length;
                        end
                    end
                end
                [curve,curveLength] = threeArcs(p,target,travel);
                if ~isempty(curve) && curveLength < shortest
                    finalLine = arc(curve(1:3,end),travel*lead,0);
                    if free(finalLine)
                        result = [curve,finalLine(:,2:end)];
                        shortest = curveLength;
                    end
                end
            end
            if ~isempty(result), return; end
        end
    end

    function [result,shortest] = threeArcs(p,target,travel)
        % Tri tangentne kroznice; srednji lok dovoljuje tudi vzvratno voznjo.
        % Tako povezemo bliznji legi tudi pri nasprotnih orientacijah.
        result = [];
        shortest = inf;
        for turn = [-1,1]
            c0 = p(1:2)+turn*radius*[-sin(p(3));cos(p(3))];
            c2 = target(1:2)+turn*radius*[-sin(target(3));cos(target(3))];
            delta = c2-c0;
            distance = norm(delta);
            if distance < 1e-9 || distance > 4*radius, continue; end
            normal = [-delta(2);delta(1)]/distance;
            height = sqrt(max(0,4*radius^2-distance^2/4));
            for side = [-1,1]
                center = (c0+c2)/2+side*height*normal;
                phi0 = atan2(center(2)-c0(2),center(1)-c0(1))+turn*pi/2;
                phi1 = atan2(c2(2)-center(2),c2(1)-center(1))-turn*pi/2;
                for middle = [travel,-travel]
                    angles = mod([travel*turn*(phi0-p(3)), ...
                        -middle*turn*(phi1-phi0), ...
                        travel*turn*(target(3)-phi1)],2*pi);
                    angles(angles > 2*pi-1e-7) = 0;
                    length = radius*sum(angles);
                    if length >= min(shortest,1.5), continue; end
                    b0 = arc(p,travel*radius*angles(1),turn/radius);
                    b1 = arc(b0(1:3,end),middle*radius*angles(2),-turn/radius);
                    b2 = arc(b1(1:3,end),travel*radius*angles(3),turn/radius);
                    b0(5,:) = travel; b1(5,:) = middle; b2(5,:) = travel;
                    candidate = [b0,b1(:,2:end),b2(:,2:end)];
                    if free(candidate)
                        result = candidate;
                        shortest = length;
                    end
                end
            end
        end
    end

    function push(id, priority)
        heapSize = heapSize+1;
        cursor = heapSize;
        while cursor > 1
            parent = floor(cursor/2);
            if heap(2,parent) <= priority, break; end
            heap(:,cursor) = heap(:,parent);
            cursor = parent;
        end
        heap(:,cursor) = [id;priority];
    end

    function id = pop()
        id = heap(1,1);
        last = heap(:,heapSize);
        heapSize = heapSize-1;
        cursor = 1;
        while 2*cursor <= heapSize
            child = 2*cursor;
            if child < heapSize && heap(2,child+1) < heap(2,child)
                child = child+1;
            end
            if last(2) <= heap(2,child), break; end
            heap(:,cursor) = heap(:,child);
            cursor = child;
        end
        heap(:,cursor) = last;
    end
end

function segment = arc(p, length, curvature)
s = linspace(0,length,max(2,ceil(abs(length)/0.008)+1));
phi = p(3)+curvature*s;
if abs(curvature) < 1e-9
    xy = p(1:2)+[cos(p(3));sin(p(3))]*s;
else
    xy = p(1:2)+[sin(phi)-sin(p(3)); ...
        cos(p(3))-cos(phi)]/curvature;
end
segment = [xy;wrapToPi(phi);curvature+zeros(size(s)); ...
    sign(length)+zeros(size(s))];
end

function a = pathDirection(path, tail)
if isempty(path), a = tail(4:5,1); else, a = path(4:5,1); end
end

function distance = gridDistances(rect,goal,shape,resolution)
% 2D Dijkstra da hevristiko za obvoz ovir; orientacijo resuje glavno iskanje.
[x,y] = ndgrid((0:shape(1)-1)*resolution,(0:shape(2)-1)*resolution);
points = [x(:).';y(:).'];
blocked = false(1,numel(x));
for b = rect
    axes = [cos(b(5)),sin(b(5));-sin(b(5)),cos(b(5))];
    local = abs(axes*(points-b(1:2)));
    blocked = blocked | all(local < b(3:4)+0.03,1);
end
blocked = reshape(blocked,shape);
xy = min(max(round(goal(1:2)/resolution)+1,1),shape(:));
distance = inf(shape);
distance(xy(1),xy(2)) = 0;
pending = distance;
while true
    [value,id] = min(pending(:));
    if ~isfinite(value), break; end
    pending(id) = inf;
    [i,j] = ind2sub(shape,id);
    for di = -1:1
        for dj = -1:1
            a = i+di; b = j+dj;
            if a < 1 || a > shape(1) || b < 1 || b > shape(2), continue; end
            if blocked(a,b), continue; end
            next = value+resolution*hypot(di,dj);
            if next < distance(a,b)
                distance(a,b) = next;
                pending(a,b) = next;
            end
        end
    end
end
end
