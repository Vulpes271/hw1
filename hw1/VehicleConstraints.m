classdef VehicleConstraints < handle
    properties
        vMax = inf; % Maximum velocity constraint (in m/s)
        aMax = inf; % Maximum acceleration constraint (in m/s^2)
        L = 0.043; % Distance between the wheels (in m)
    end

    properties
        vL = 0; % Velocity of the left wheel (in m/s)
        vR = 0; % Velocity of the right wheel (in m/s)
    end

    methods
        function obj = VehicleConstraints(vMax, aMax)
            %Vehicle constraints
            %
            %Inputs
            % vMax: Maximum velocity constraint (in m/s)
            % aMax: Maximum acceleration constraint (in m/s^2)
            if nargin < 1 || isempty(vMax)
                vMax = inf;
            end
            if nargin < 2 || isempty(aMax)
                aMax = inf;
            end
            obj.vMax = vMax;
            obj.aMax = aMax;
        end

        function reset(obj)
            %Reset
            obj.vL = 0;
            obj.vR = 0;
        end

        function [v, w] = update(obj, v, w, T)
            %Update (check and apply constraints)
            %
            %Inputs
            % v: Linear velocity (in m/s)
            % w: Angular velocity (in rad/s)
            % T: Sampling time (in s)
            %
            %Outputs
            % v: Linear velocity (in m/s)
            % w: Angular velocity (in rad/s)
            vL = v - obj.L*w/2;
            vR = v + obj.L*w/2;
            
            % Limited wheel acceleration
            vL = min(max(vL, obj.vL-obj.aMax*T), obj.vL+obj.aMax*T);
            vR = min(max(vR, obj.vR-obj.aMax*T), obj.vR+obj.aMax*T);
            % Limited wheel velocity
            vL = min(max(vL, -obj.vMax), obj.vMax);
            vR = min(max(vR, -obj.vMax), obj.vMax);
            % Velocities
            obj.vL = vL;
            obj.vR = vR;

            v = (vL + vR)/2;
            w = (vR - vL)/obj.L;
        end
    end
end