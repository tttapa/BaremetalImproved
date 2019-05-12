function [ result_u, result_y_int_increment ] = GenerateAttitudeController(s)
%GENERATE_ATTITUDE_CONTROLLER
%   Code generator for the attitude controller. This function creates syms that
%   can be used to replace tags with generated code in AttitudeCodegen.cpp.
%
%   @param  s
%           The result of GetParamsAndMatrices().
%
%   @return result_u
%           LQR control signal, calculated from the syms 'vector__ref',
%           'vector__y_int' and 'vector__x_hat'.
%   @return result_y_int_increment
%           Increment to be made to the integral windup, calculated from the
%           syms 'vector__ref' and 'vector__x_hat'.
%

% Round matrices
G = round(s.att.lqr.G, 8);
K = round(s.att.lqi.K, 8);
    
% Create syms
r     = sym('vector__ref',   [3, 1], 'real');
x_hat = sym('vector__x_hat', [9, 1], 'real');
y_int = sym('vector__y_int', [3, 1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:9);
u_eq = eq(10:end);

% Calculate error
x_diff = x_hat - x_eq;
y = s.att.Cd * x_hat;
y_err = r - y;
result_y_int_increment = y_err * s.att.Ts;
err = [ x_diff; y_int ];

% Calculate output
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end
