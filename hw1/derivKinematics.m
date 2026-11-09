function dy = derivKinematics(t, y, vs, ws, D)
%Robot kinematics
%
%Inputs
% t:  Time (in s)
% y:  States [x; y; phi; gamma]
% vs: Linear velocity of the front part (in m/s)
% ws: Angular velocity of the front part (in rad/s)
% D:  Distance between the rear axle and the center of the front part (in m)
%
%Outputs
% dy: State derivatives
dy = [vs*cos(y(4))*cos(y(3)); ...
      vs*cos(y(4))*sin(y(3)); ...
      vs*sin(y(4))/D; ...
      ws-vs*sin(y(4))/D];