function q = updateVehicle(q, vs, ws, T)
%Update vehicle configuration
%
%Inputs
% q:  Current vehicle configuration [x; y; phi; gamma] (in m, m, rad and rad) at time step k
% vs: Linear velocity of the front part (in m/s) at time step k
% ws: Angular velocity of the front part (in rad/s) at time step k
% T:  Sampling time (in s)
%
%Outputs
% q: Updated vehicle configuration [x; y; phi; gamma] (in m, m, rad and rad) at time step k + 1
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
sol = ode45(@(t, y) derivKinematics(t, y, vs, ws, 0.1207), [0, T], q, opts);
q = deval(T, sol);
q(3:4) = wrapToPi(q(3:4));