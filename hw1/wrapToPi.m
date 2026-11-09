function x = wrapToPi(x)
%Wrap angle to [-pi, pi]
%
%Inputs
% x: Angle (in rad)
%
%Outputs
% x: Wrapped angle (in rad)
x = atan2(sin(x), cos(x));
