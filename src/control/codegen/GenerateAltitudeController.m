function [ result_u, result_y_int_increment ] = GenerateAltitudeController(s)
%GENERATE_ALTITUDE_CONTROLLER
%   Code generator for the altitude controller. This function creates syms that
%   can be used to replace tags with generated code in AltitudeCodegen.cpp.
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

% TODO: should round?
% Round matrices
G = round(s.alt.lqr.G, 8);
K = round(s.alt.lqi.K, 8);

% Create syms
r     = sym('vector__ref',   [1, 1], 'real');
x_hat = sym('vector__x_hat', [3, 1], 'real');
y_int = sym('vector__y_int', [1, 1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:3);
u_eq = eq(4:end);

% Calculate error
x_err = x_hat - x_eq;
y = s.alt.Cd * x_hat;
y_err = r - y;
result_y_int_increment = y_err * s.alt.Ts;
err = [ x_err; y_int ];

% Calculate output (LQR with integral action)
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end