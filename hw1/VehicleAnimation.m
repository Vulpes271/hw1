classdef VehicleAnimation < handle
    properties
        decimation = 1; % Drawing decimation (if 0, animation is disabled)
    end

    properties (SetAccess=private)
        key = ''; % Last pressed key
        count = 0; % Key press count
    end

    properties (SetAccess=private)
        chrono;
        map;

        hFig;
        hAx;
        hVs;
        hWs;
        hVL;
        hVR;
        hC;
        hPath = Line.empty();
        hVehicle = Vehicle.empty();
        hLabels;

        reqClose = false;
        finished = false;

        vLp = nan;
        vRp = nan;

        src;
        dst;
        gVehicle = Vehicle.empty();

        goalId;

        active = 0;

        colors = lines(11);
    end

    methods
        function obj = VehicleAnimation(chrono, map)
            %Vehicle animation
            %
            % Keyboard key ESCAPE stops the simulation.
            % Keyboard key SHIFT+ESCAPE also closes the animation.
            %
            %Inputs
            % chrono:      Chrono object
            % map:         Map
            obj.chrono = chrono;
            obj.map = map;

            obj.hFig = figure('Units', 'normalized', 'Position', [0.1, 0.1, 0.7, 0.6]);

            subplot('Position', [0.07, 0.7, 0.25, 0.25]);
            obj.hVs = Line('b', '-'); obj.hWs = Line('r', '-'); xlim([0, 30]);
            ylabel('v_s [m/s], \omega_s [rad/s]'); legend('v_s', '\omega_s'); box;

            subplot('Position', [0.07, 0.4, 0.25, 0.25]);
            obj.hVL = Line('b', '-'); obj.hVR = Line('r', '-'); xlim([0, 30]);
            ylabel('v_L, v_R [m/s]'); legend('v_L', 'v_R'); box;
            
            subplot('Position', [0.07, 0.1, 0.25, 0.25]);
            obj.hC = Line('r', '-'); xlim([0, 30]); ylim([-0.2, 1.2]); set(gca, 'YTick', [0, 1], 'YTickLabel', {'No', 'Yes'});
            xlabel('t [s]'); ylabel('Collision'); box;
            
            obj.hAx = subplot('Position', [0.33, 0.1, 0.65, 0.85]);
            axis('equal'); axis('manual'); zlim([-1, 1]);
            xlabel('x [m]'); ylabel('y [m]'); box;

            % Obstacles
            obstacles(1,3) = polyshape;
            for i = 1:length(map.obstacles)
                patch('XData', map.obstacles{i}(1,[1:end,1]), ...
                                'YData', map.obstacles{i}(2,[1:end,1]), ...
                                'ZData', -0.5*ones(1,5), ...
                                'FaceColor', [0.8, 0.8, 0.8]);
                obstacles(i) = polyshape(map.obstacles{i}(1,:), map.obstacles{i}(2,:));
            end

            % Parking poses
            for i = 1:size(map.parkingPoses, 2)
                obj.gVehicle(i) = Vehicle(obj.hAx);
                obj.gVehicle(i).sketch('k');
                obj.gVehicle(i).setConfiguration(map.parkingPoses(:,i));
            end
            
            % Parking areas
            for i = 1:length(map.parkingAreas)
                obj.dst(i) = patch('XData', map.parkingAreas{i}(1,[1:end,1]), ...
                      'YData', map.parkingAreas{i}(2,[1:end,1]), ...
                      'ZData', -0.51*ones(1,5), ...
                      'FaceColor', 'none', 'EdgeColor', 'none', 'Parent', obj.hAx);
            end

            % Starting area
            obj.src = patch('XData', map.startingArea(1,[1:end,1]), ...
                  'YData', map.startingArea(2,[1:end,1]), ...
                  'ZData', -0.51*ones(1,5), ...
                  'FaceColor', [0.95, 1.0, 1.0], 'EdgeColor', 'none', 'Parent', obj.hAx);

            % Labels
            for i = 1:size(map.parkingPoses, 2)
                p = map.parkingPoses(:,i);
                if cos(p(3)) < -0.5
                    ha = 'right';
                elseif cos(p(3)) > 0.5
                    ha = 'left';
                else
                    ha = 'center';
                end
                obj.hLabels(i) = text(p(1), p(2), sprintf('P%d', i), 'VerticalAlignment', 'middle', 'HorizontalAlignment', ha);
            end

            axis([0, map.width, 0, map.height]);
            
            s = dbstack('-completenames');
            if ~contains(s(end).file, 'LiveEditorEvaluation')
                set(obj.hFig, 'KeyPressFcn', @obj.handleKey, 'CloseRequestFcn', @obj.handleCloseReq);
            end
        end

        function spawnVehicle(obj, q0, goalId)
            %Spawn a new vehicle
            %
            %Inputs
            % q0:     Initial configuration
            % goalId: ID of the goal parking spot
            if obj.active > 0
                obj.hVehicle(obj.active).gray();
            end
            obj.active = obj.active + 1;

            obj.hPath(obj.active) = Line(obj.colors(obj.active,:), '-');
            obj.hVehicle(obj.active) = Vehicle();
            obj.hVehicle(obj.active).setConfiguration(q0);
            obj.hVehicle(obj.active).setColor(obj.colors(obj.active,:));

            obj.gVehicle(goalId).sketch(obj.colors(obj.active,:));
            set(obj.hLabels(goalId), 'String', sprintf('P%d_{%d}', goalId, obj.active));
            
            obj.goalId = goalId;
            set(obj.dst(obj.goalId), 'FaceColor', [1, 1, 0.9]);
            
            obj.hVs.reset();
            obj.hWs.reset();
            obj.hVL.reset();
            obj.hVR.reset();
            obj.hC.reset();
        end

        function update(obj, q, vs, ws, collision, inside)
            %Update animation
            %
            %Inputs
            % q:         Vehicle configuration [x; y; phi; gamma] at k, i.e. t
            % vs:        Linear velocity of the front part at k, i.e. t
            % ws:        Angular velocity of the front part at k, i.e. t
            % collision: Is the vehicle in collision?
            % inside:    Is the vehicle inside the goal parking area?
            obj.hVehicle(obj.active).setConfiguration(q);
            obj.hPath(obj.active).addPoint(q);
            t = obj.chrono.t;

            % Is the destination area reached?
            if inside
                set(obj.dst(obj.goalId), 'FaceColor', [0.8, 1, 0.8]);
            else
                set(obj.dst(obj.goalId), 'FaceColor', [1, 1, 0.9]);
            end

            vL = vs - obj.hVehicle(obj.active).L*ws/2;
            vR = vs + obj.hVehicle(obj.active).L*ws/2;

            % aL = (vL - obj.vLp)/obj.chrono.T;
            % aR = (vR - obj.vRp)/obj.chrono.T;

            obj.vLp = vL;
            obj.vRp = vR;
            
            obj.hVs.addPoint([t, vs]); obj.hWs.addPoint([t, ws]); obj.hVs.move();
            obj.hVL.addPoint([t, vL]); obj.hVR.addPoint([t, vR]); obj.hVL.move();
            obj.hC.addPoint([t, collision]); obj.hC.move();

            if obj.decimation > 0 && mod(obj.chrono.k, obj.decimation) == 0
                drawnow;
            end
        end

        function finish(obj)
            %Finish animation
            obj.hVs.fromZero();
            obj.hVL.fromZero();
            obj.hC.fromZero();
            drawnow;
            set(obj.hFig, 'CloseRequestFcn', 'closereq');
            if obj.reqClose
                close(obj.hFig);
            end
            obj.finished = true;
        end

        function data = collect(obj, q0, qStart, vehicleId, goalId, goalPose, qEnd, tEnd, inside, noise)
            %Collect data
            %
            %Inputs
            % q0:        Initial desired configuration
            % qStart:    Initial configuration
            % vehicleId: Vehicle ID
            % goalId:    ID of the goal parking spot
            % goalPose:  Desired pose in the goal parking spot
            % qEnd:      Final configuration
            % tEnd:      Final time (in s)
            % inside:    Is inside the goal parking area?
            % noise:     Is noise enabled?
            %
            %Outputs
            % data: Structured data
            data = struct();
            data.q0 = q0;
            data.qStart = qStart;
            data.vehicleId = vehicleId;
            data.goalId = goalId;
            data.goalPose = goalPose;
            data.qEnd = qEnd;
            data.tEnd = tEnd;
            data.inside = inside;
            data.noise = noise;
            data.t = get(obj.hVs.line, 'XData');
            data.vs = get(obj.hVs.line, 'YData');
            data.ws = get(obj.hWs.line, 'YData');
            data.vl = get(obj.hVL.line, 'YData');
            data.vr = get(obj.hVR.line, 'YData');
            data.c = get(obj.hC.line, 'YData');
            data.x = get(obj.hPath(obj.active).line, 'XData');
            data.y = get(obj.hPath(obj.active).line, 'YData');
        end
    end

    methods (Access=private)
        function handleKey(obj, src, event)
            %Handle key press event
            %
            %Inputs
            % src:   Source object
            % event: Event data
            obj.key = event.Key;
            obj.count = obj.count + 1;
            if strcmp(obj.key, 'escape')
                obj.chrono.stop = true;
                if any(strcmp(event.Modifier, 'shift'))
                    if obj.finished
                        close(obj.hFig);
                    else
                        obj.reqClose = true;
                    end
                end
            end
        end

        function handleCloseReq(obj, src, event)
            %Handle close request event
            %
            %Inputs
            % src:   Source object
            % event: Event data
            obj.chrono.stop = true;
            if obj.reqClose
                delete(src);
            else
                obj.reqClose = true;
            end
        end
    end
end