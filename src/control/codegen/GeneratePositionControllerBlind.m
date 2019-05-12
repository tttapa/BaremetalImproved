function [ result_u, result_y_int_increment ] = GeneratePositionControllerBlind(s)
%GENERATE_POSITION_CONTROLLER_BLIND
%   Code generator for the blind position controller. This function creates syms
%   that can be used to replace tags with generated code in PositionCodegen.cpp.
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
G = round(s.posBlind.lqr.G, 8);
K = round(s.posBlind.lqi.K, 8);

% Create syms
r     = sym('vector__ref',   [2, 1], 'real');
r     = [zeros(2, 1); r];
x_hat = sym('vector__x_hat', [6, 1], 'real');
y_int = sym('vector__y_int', [2, 1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:6); 
u_eq = eq(7:end); 

% Calculate error
x_err = x_hat - x_eq;
y = s.posBlind.Cdfull * x_hat;
y_err = r(3:4) - y(3:4);
result_y_int_increment = y_err * s.posBlind.Ts;
err = [ x_err; y_int ];

% Calculate output
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end


    