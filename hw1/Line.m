classdef Line < handle
    properties
        line; % Line object
    end
    
    methods
        function obj = Line(color, style, marker)
            % Line
            %
            %Inputs
            % color: Color (default: 'k')
            % style: Style (default: '-')
            % marker: Marker (default: 'none')
            if nargin<1 || isempty(color)
                color = 'k';
            end
            if nargin<2 || isempty(style)
                style = '-';
            end
            if nargin<3 || isempty(marker)
                marker = 'none';
            end
            obj.line = line('XData', [], 'YData', [], ...
                'Color', color, 'LineStyle', style, 'Marker', marker);
        end
        
        function addPoint(obj, p)
            %Add point
            %
            %Inputs
            % p: Point(s) ([x; y] in each column)
            if size(p, 1) == 1
                p = p(:);
            end
            set(obj.line, 'XData', [get(obj.line, 'XData'), p(1,:)], ...
                          'YData', [get(obj.line, 'YData'), p(2,:)]);
        end
        
        function move(obj)
            %Move axis
            x = get(obj.line, 'XData');
            e = x(end);
            a = get(obj.line, 'Parent');
            l = get(a, 'XLim');
            if e > l(end)
                l = l + diff(l);
                set(a, 'XLim', l);
            end
        end
        
        function fromZero(obj)
            %Show axis from zero to the end
            x = get(obj.line, 'XData');
            if ~isempty(x) && length(x) > 1
                a = get(obj.line, 'Parent');
                set(a, 'XLim', [0, x(end)]);
            end
        end

        function reset(obj)
            %Clear data
            set(obj.line, 'XData', [], ...
                          'YData', []);
        end
    end
end
