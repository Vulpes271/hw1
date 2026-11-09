function [v, w, stop] = manualDrive(key, count)
%Manual drive
%
% Keyboard keys for setting v (default: 0.1 m/s) and w (default: 0.5 rad/s):
%
%  key   |   u      i      o  
% -----  |  -----  -----  -----
% v   w  |  +V +W  +V  0  +V -W
%
%  key   |    j      k      l  
% -----  |  -----  -----  -----
% v   w  |  0  +W  0   0  0  -W
%
%  key   |    m      ,      .  
% -----  |  -----  -----  -----
% v   w  |  -V +W  -V  0  -V -W
%
% Keys q/a/y: Increase/reset/decrease V and W
% Keys w/s/x: Increase/reset/decrease V
% Keys e/d/c: Increase/reset/decrease W
% Key f:      Print current values of V and W
% Key g:      Emit stop signal and set velocities to zero
% Escape key: Stop the simulation
%
%Inputs
% key:   Pressed key
% count: Key press count
%
%Outputs
% v: Linear velocity, in m/s 
% w: Angular velocity, in rad/s
% stop: Emit stop signal

persistent V W last;
if isempty(V), V = 0.1; end
if isempty(W), W = 0.5; end
if isempty(last), last = count; end

stop = false;

switch key
    case 'k' % Stop
        v = 0; w = 0;
    case 'i' % Forward
        v = V; w = 0;
    case 'comma' % Backward
        v = -V; w = 0;
    case 'j' % Left
        v = 0; w = W;
    case 'l' % Right
        v = 0; w = -W;
    case 'u' % Forward left
        v = V; w = W;
    case 'o' % Forward right
        v = V; w = -W;
    case 'm' % Backward left
        v = -V; w = W;
    case 'period' % Backward right
        v = -V; w = -W;
    otherwise
        v = 0; w = 0;
        if count ~= last
            switch key
                case 'g' % Emit stop signal
                    stop = true;

                case 'q' % Increase all velocities
                    V = V*1.1;
                    W = W*1.1;
                    fprintf('v = %g m/s, w = %g rad/s\n', V, W);
                case 'a' % Reset all velocities
                    V = 0.1;
                    W = 0.5;
                    fprintf('v = %g m/s, w = %g rad/s\n', V, W);
                case 'y' % Decrease all velocities
                    V = V*0.9;
                    W = W*0.9;
                    fprintf('v = %g m/s, w = %g rad/s\n', V, W);
                
                case 'w' % Increase linear velocity
                    V = V*1.1;
                    fprintf('v = %g m/s\n', V);
                case 's' % Reset linear velocity
                    V = 0.1;
                    fprintf('v = %g m/s\n', V);
                case 'x' % Decrease linear velocity
                    V = V*0.9;
                    fprintf('v = %g m/s\n', V);

                case 'e' % Increase angular velocity
                    W = W*1.1;
                    fprintf('w = %g rad/s\n', W);
                case 'd' % Reset angular velocity
                    W = 0.5;
                    fprintf('w = %g rad/s\n', W);
                case 'c' % Decrease angular velocity
                    W = W*0.9;
                    fprintf('w = %g rad/s\n', W);

                case 'f' % Print all velocities
                    fprintf('v = %g m/s, w = %g rad/s\n', V, W);
            end
        end
end

last = count;
