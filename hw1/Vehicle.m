classdef Vehicle < handle
    properties (Access=protected)
        main; % Main object
        cart; % Front part
        body; % Body
        z = -0.1; % Z-offset
        cartBody; % Body of the front part
        cartLine; % Line sensor on the front part
        color = [0, 0.8, 0]; % Main vehicle color
    end
    
    properties (Constant)
        L = 0.043;
        D = 0.1207;
        B = 0.08;
        R = 0.016;
        T = 0.006;
    end
    
    methods
        function obj = Vehicle(parent)
            %Vehicle
            %
            %Inputs
            % parent: Parent object (default is current axes)
            obj.main = hgtransform();
            if nargin>0 && ~isempty(parent)
                set(obj.main, 'Parent', parent)
            end
            
            O = sqrt((obj.L/2)^2 + obj.R^2);
            U = O + obj.T;

            W = 2*U + 2*obj.T;
            H = obj.D + U + obj.R + 2*obj.R;
            I = 2*obj.R;

            H = 0.193;
            W = 0.096;
            I = 0.023;

            E = (obj.L/2-obj.T);
            F = sqrt(O^2 - E^2);

            obj.cart = hgtransform('Parent', obj.main);
            t = linspace(0, 2*pi, 50);
            obj.body = patch('XData', [0, 1, 1, 0, 0]*H-I, ...
                  'YData', [-1, -1, 1, 1, -1]*W/2, ...
                  'ZData', ones(1,5)*0, ...
                  'Parent', obj.main, 'FaceColor', obj.color); % main body
            patch('XData', ([0, 1, 1, 0, 0]-0.5)*2*obj.R, ...
                  'YData', [-1, -1, 1, 1, -1]*obj.T/2-obj.B/2, ...
                  'ZData', ones(1,5)*0.001, ...
                  'Parent', obj.main, 'FaceColor', [0, 0, 0]); % RR wheel
            patch('XData', ([0, 1, 1, 0, 0]-0.5)*2*obj.R, ...
                  'YData', [-1, -1, 1, 1, -1]*obj.T/2+obj.B/2, ...
                  'ZData', ones(1,5)*0.001, ...
                  'Parent', obj.main, 'FaceColor', [0, 0, 0]); % RL wheel
            patch('XData', U*cos(t)+obj.D, ...
                  'YData', U*sin(t), ...
                  'ZData', ones(size(t))*0.001, ...
                  'Parent', obj.main, 'FaceColor', [1, 1, 1]); % front
            obj.cartBody = patch('XData', ([0, 1, 1, 0, 0]-0.5)*2*F, ...
                  'YData', [-1, -1, 1, 1, -1]*E, ...
                  'ZData', ones(1,5)*0, ...
                  'Parent', obj.cart, 'FaceColor', [1, 0.5, 0]); % cart
            patch('XData', [-1, 1, 1, -1, -1]*obj.R, ...
                  'YData', [-1, -1, 1, 1, -1]*obj.T/2+obj.L/2, ...
                  'ZData', ones(1,5)*0, ...
                  'Parent', obj.cart, 'FaceColor', [0, 0, 0]); % FL wheel
            patch('XData', [-1, 1, 1, -1, -1]*obj.R, ...
                  'YData', -[-1, -1, 1, 1, -1]*obj.T/2-obj.L/2, ...
                  'ZData', ones(1,5)*0, ...
                  'Parent', obj.cart, 'FaceColor', [0, 0, 0]); % FR wheel
            obj.cartLine = patch('XData', -[0, 1, 1, 0, 0]*obj.T + F - obj.T, ...
                  'YData', [-1, -1, 1, 1, -1]*(E - obj.T), ...
                  'ZData', ones(1,5)*0.001, ...
                  'Parent', obj.cart, 'FaceColor', [1, 0, 0]); % line sensor
            obj.setConfiguration([0; 0; 0; 0]);
        end
        
        function setConfiguration(obj, q)
            %Set vehicle configuration
            %
            %Inputs
            % q: Vehicle configuration [x; y; phi; gamma]
            if length(q)<4
                q(4) = 0;
            end
            set(obj.main, 'Matrix', makehgtform( ...
                'translate', [q(1), q(2), obj.z], ...
                'zrotate', q(3)));
            set(obj.cart, 'Matrix', makehgtform( ...
                'translate', [obj.D, 0, 0.01], ...
                'zrotate', q(4)));
        end
        
        function outline = getOutline(obj)
            %Get vehicle outline
            %
            %Outputs
            % outline: Vehicle outline in world coordinates
            M = get(obj.main, 'Matrix');
            p = [get(obj.body, 'XData'), get(obj.body, 'YData')].';
            outline = M(1:2,1:2)*p(:,1:end-1) + M(1:2,end);
        end
        
        function sketch(obj, color)
            %Make a sketch of the vehicle
            %
            %Inputs
            % color: Set outline color of the sketch (default: 'b')
            if nargin < 2 || isempty(color)
                color = 'b';
            end
            o = findobj(obj.main, 'Type', 'Patch');
            for i = 1:length(o)
                set(o(i), 'FaceColor', 'none', 'EdgeColor', color, 'LineStyle', '-');
            end
            o = findobj(obj.cart, 'Type', 'Patch');
            for i = 1:length(o)
                set(o(i), 'FaceColor', 'none', 'EdgeColor', 'none');
            end
            obj.z = 0;
        end

        function setColor(obj, color)
            %Set main vehicle color
            %
            %Inputs
            % color: Color as numeric triplet or string code
            if isnumeric(color)
                obj.color = color;
            else
                switch color
                    case 'r'
                        obj.color = [1, 0.8, 0.8];
                    case 'g'
                        obj.color = [0.8, 1, 0.8];
                    case 'b'
                        obj.color = [0.8, 0.8, 1];
                    case 'c'
                        obj.color = [0.8, 1, 1];
                    case 'm'
                        obj.color = [1, 0.8, 1];
                    case 'y'
                        obj.color = [1, 1, 0.8];
                    otherwise
                        obj.color = [0, 0, 0];
                end
            end
        end

        function gray(obj, gray)
            %Show vehicle in gray
            %
            %Inputs
            % gray: Show in gray (default: true)
            if nargin < 2 || isempty(gray)
                gray = true;
            end
            if gray
                set(obj.body, 'FaceColor', [0.8, 0.8, 0.8]);
                set(obj.cartBody, 'FaceColor', [0.5, 0.5, 0.5]);
                set(obj.cartLine, 'FaceColor', [0.8, 0.8, 0.8]);
            else
                set(obj.body, 'FaceColor', obj.color);
                set(obj.cartBody, 'FaceColor', [1, 0.5, 0]);
                set(obj.cartLine, 'FaceColor', [1, 0, 0]);
            end
        end
    end
end
