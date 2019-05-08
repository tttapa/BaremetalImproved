function [result_u, result_y_int_inc] = GenerateAltitudeController(s)
%myFun - Code generator for the altitude controller
%
% Syntax: u = GenerateAltitudeController(s)
%
% `s` is the result of GetParamsAndMatrices that contains the proportional and 
% integral controller matrix `lqi.K`

% Round matrices
G = round(s.alt.lqr.G, 8);
K = round(s.alt.lqi.K, 8);

% Create syms
r     = sym('vector__ref',   [1, 1], 'real');
y_int = sym('vector__y_int', [1, 1], 'real');
x_hat = sym('vector__x_hat', [3, 1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:3);
u_eq = eq(4:end);

% Calculate error
x_err = x_hat - x_eq;
y = s.alt.Cd * x_hat;
y_err = r - y;
result_y_int_inc = y_err * s.alt.Ts;
err = [ x_err; y_int ];

% Calculate output
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end