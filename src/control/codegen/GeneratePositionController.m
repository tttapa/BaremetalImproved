function [result_u, result_y_int_inc] = GeneratePositionController(s)
%myFun - Code generator for the navigation controller
%
% Syntax: u = GeneratePositionController(s)
%
% `s` is the result of GetParamsAndMatrices that contains the proportional and 
% integral controller matrix `lqi.K`

% Round matrices
G = round(s.pos.lqr.G, 8);
K = round(s.pos.lqi.K, 8);

% Create syms
r     = sym('vector__ref',   [2, 1], 'real');
r     = [zeros(2, 1); r];
y_int = sym('vector__y_int', [2, 1], 'real');
x_hat = sym('vector__x_hat', [6, 1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:6); 
u_eq = eq(7:end); 

% Calculate error
x_err = x_hat - x_eq;
y = s.pos.Cd * x_hat;
y_err = r(3:4) - y(3:4);
result_y_int_inc = y_err * s.pos.Ts;
err = [ x_err; y_int ];

% Calculate output
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end


    