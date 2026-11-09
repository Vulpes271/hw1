classdef Chrono < handle
    properties
        k = -1; % Current time step
        T = 0.1; % Sampling time (in s)
        tSim = inf; % Simulation time (in s)
        realtime = false; % Real-time simulation
        stop = false; % Should stop?
    end

    properties (Dependent, SetAccess=private)
        t; % Current time (in s)
    end

    properties (Access=private)
        t0; % Timer
    end

    methods
        function obj = Chrono(T)
            %Chrono
            %
            %Inputs
            % T: Sampling time (in s, default is 0.1)
            if nargin < 1
                T = 0.1;
            end
            obj.T = T;
            obj.init()
        end

        function init(obj)
            %Init
            %
            % Starts the timer.
            obj.t0 = tic();
        end

        function ok = loop(obj)
            %Loop
            %
            %Outputs
            % ok: Should the loop continue?
            if obj.realtime, pause(obj.t-toc(obj.t0)); end
            ok = ~obj.stop && obj.t < obj.tSim;
            if ok, obj.k = obj.k + 1; end
        end

        function t = get.t(obj)
            t = obj.k*obj.T;
        end

        function reset(obj)
            %Reset timer
            obj.k = -1;
            obj.stop = false;
            obj.init();
        end
    end
end