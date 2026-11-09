classdef Map < handle
    properties
        obstacles = {}; % Outlines of static obstacles
        parkingAreas = {}; % Outlines of the parking areas
        parkingPoses = []; % Poses of the parking spots (goal [x; y; phi] in each column)
        
        vehicleObstacles = {}; % Outlines of the vehicles that are considered as obstacles
        vehiclePoses = []; % Poses of the vehicles that are considered as obstacles
        
        startingArea = []; % Outline of the starting area

        H = 0.1930; % Vehicle length, in m
        W = 0.0960; % Vehicle width, in m
        I = 0.023; % Offset of the rear axle, in m
        G = 0.02; % Margin around each parking spot, in m
        T = 0.02; % Wall thickness, in m
    end

    properties (SetAccess=protected)
        obst = polyshape; % All obstacles that are considered for collision detection
    end

    properties (Dependent)
        width; % Width of the entire map, in m
        height; % Height of the entire map, in m
        nObst; % Number of static obstacles
    end

    methods
        function width = get.width(obj)
            width = obj.H*6 + obj.G*10 + 2*obj.T;
        end

        function height = get.height(obj)
            height = obj.W*10 + obj.G*8 + 2*obj.T;
        end

        function nObst = get.nObst(obj)
            nObst = length(obj.obstacles);
        end

        function obj = Map()
            % Map

            % Static obstacles
            obj.obstacles{1} = [[0, 1, 1, 0]*obj.width; ...
                                [0, 0, 1, 1]*obj.T]; % Bottom
            obj.obstacles{2} = [[0, 1, 1, 0]*obj.width; ...
                                [0, 0, 1, 1]*obj.T + obj.height - obj.T]; % Top
            obj.obstacles{3} = [[0, 1, 1, 0]*obj.T; ...
                                [0, 0, 1, 1]*(obj.height-2*obj.T) + obj.T]; % Left
            obj.obstacles{4} = [[0, 1, 1, 0]*(obj.H+2*obj.G)*2 + obj.T; ...
                                [0, 0, 1, 1]*(obj.W+2*obj.G) + obj.T]; % Bottom left
            obj.obstacles{5} = [[0, 1, 1, 0]*(obj.H+2*obj.G) + obj.T; ...
                                [0, 0, 1, 1]*(obj.W+obj.G) + (obj.W+2*obj.G) + obj.T]; % Above bottom left
            obj.obstacles{6} = [[0, 1, 1, 0]*(obj.H+2*obj.G) + obj.width - obj.H - 2*obj.G - obj.T; ...
                                [0, 0, 1, 1]*(obj.W+2*obj.G) + obj.T]; % Bottom right
            obj.obstacles{7} = [[0, 1, 1, 0]*obj.T + obj.width - obj.T; ...
                                [0, 0, 1, 1]*(obj.height-2*obj.T) + obj.T]; % Right

            % Parking poses
            obj.parkingPoses = [obj.T + obj.H + obj.G - obj.I, obj.T + obj.W*4 + obj.G*6 + obj.W/2, pi; ...
                obj.T + obj.G + obj.I, obj.T + obj.W*3 + obj.G*5 + obj.W/2, 0; ...
                obj.T + obj.H + obj.G - obj.I, obj.T + obj.W*2 + obj.G*4 + obj.W/2, pi; ...
                obj.T + obj.H*2 + obj.G*5 + obj.I, obj.T + (obj.W/2+obj.G), 0; ...
                obj.T + obj.H*4 + obj.G*6 - obj.I, obj.T + (obj.W/2+obj.G), pi; ...
                obj.T + obj.H*4 + obj.G*7 + obj.I, obj.T + (obj.W/2+obj.G), 0; ...
                obj.T + obj.H*6 + obj.G*9 - obj.I, obj.T + obj.W + obj.G*3 + obj.W/2, pi; ...
                obj.T + obj.H*5 + obj.G*9 + obj.I, obj.T + obj.W*2 + obj.G*4 + obj.W/2, 0; ...
                obj.T + obj.H*6 + obj.G*9 - obj.I, obj.T + obj.W*3 + obj.G*5 + obj.W/2, pi; ...
                obj.T + obj.H*5 + obj.G*9 + obj.I, obj.T + obj.W*4 + obj.G*6 + obj.W/2, 0; ...
                obj.T + obj.H*3 + obj.G*5.5, obj.T + obj.W*5 + obj.G*6 - obj.I, -pi/2].';
            
            for i = 1:obj.nObst
                obj.obst(i) = polyshape(obj.obstacles{i}(1,:), obj.obstacles{i}(2,:));
            end

            % Destinations
            for i = 1:size(obj.parkingPoses,2)
                obj.parkingAreas{i} = obj.getOutline(obj.parkingPoses(:,i), obj.G);
            end

            % Starting area
            obj.startingArea = [[obj.T+obj.G, obj.width-obj.T-obj.G, obj.width-obj.T-obj.G, obj.T+obj.G]; ...
                                [obj.height-obj.T-obj.G-5*obj.W, obj.height-obj.T-obj.G-5*obj.W, obj.height-obj.T-obj.G, obj.height-obj.T-obj.G]];
        end

        function setVehiclesAsObstacles(obj, poses)
            % Set vehicle poses that are considered as obstacles
            %
            %Inputs
            % poses: Poses of the vehicles (pose [x; y; phi] in each column)
            obj.obst(obj.nObst+1:end) = [];

            obj.vehicleObstacles = {};
            obj.vehiclePoses = poses;

            for i = 1:size(poses,2)
                obj.vehicleObstacles{i} = obj.getOutline(poses(:,i));
                obj.obst(obj.nObst+i) = polyshape(obj.vehicleObstacles{i}(1,:), obj.vehicleObstacles{i}(2,:));
            end
        end

        function outline = getOutline(obj, q, m)
            % Get outline of the vehicle at the configuration q
            %
            %Inputs
            % q: Vehicle configuration [x; y; phi]
            % m: Margin around the vehicle (default is 0)
            %
            %Outputs
            % outline: Outline of the vehicle [x1 x2 ...; y1 y2 ...]
            if nargin < 3 || isempty(m)
                m = 0;
            end
            p = [[0, 1, 1, 0, 0]*(obj.H+2*m)-obj.I-m; [-1, -1, 1, 1, -1]*(obj.W/2+m)];

            M = makehgtform('translate', [q(1), q(2), 0], 'zrotate', q(3));
            outline = M(1:2,1:2)*p(:,1:end-1) + M(1:2,end);
        end

        function collision = isCollision(obj, pose)
            % Is it in collision?
            %
            %Inputs
            % pose: Vehicle pose
            %
            %Outputs
            % collision: Collision (true or false)?
            outline = obj.getOutline(pose);
            over = overlaps([polyshape(outline(1,:), outline(2,:)), obj.obst]);
            collision = any(over(1,2:end));
        end

        function inside = isInside(obj, pose, area)
            % Is inside the area?
            %
            %Inputs
            % pose: Vehicle pose
            % area: Area boundary points ([x; y] in each column)
            %
            %Outputs
            % inside: Inside (true or false)?
            outline = obj.getOutline(pose);
            inside = all(inpolygon(outline(1,:), outline(2,:), area(1,:), area(2,:)));
        end

        function q = getVehicleInStartingArea(obj, q, gammaMax)
            % Get vehicle configuration in starting area
            %
            %Inputs
            % q:        Vehicle configuration (if non-empty, it is returned unchanged, otherwise a random collision-free configuration in the starting area is generated)
            % gammaMax: Maximum gamma (default: pi)
            %
            %Outputs
            % q: Vehicle configuration in starting area
            if nargin < 2
                q = [];
            end
            if nargin < 3 || isempty(gammaMax)
                gammaMax = pi;
            end

            k = 0;
            while true
                if ~isempty(q) && (k == 0 || (obj.isInside(q, obj.startingArea) && ~obj.isCollision(q)))
                    break;
                end

                k = k + 1;
                if k > 1000
                    error('Unable to find a valid initial pose in the starting area. Abort.');
                end

                r = [min(obj.startingArea(1,:)), max(obj.startingArea(1,:));
                     min(obj.startingArea(2,:)), max(obj.startingArea(2,:));
                     -pi, pi;
                     -gammaMax, gammaMax];
                q = r(:,1)  + (r(:,2) - r(:,1)).*rand(4,1);
            end
        end
    end
end