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
r     = sym('vector__ref', [4,1], 'real');
x_hat = sym('vector__x_hat', [10, 1], 'real');
y_int = sym('vector__y_int', [3,  1], 'real');

% Calculate equilibrium
eq = G * r;
x_eq = eq(1:10);
u_eq = eq(11:end);

% Calculate error
x_diff = x_hat - x_eq;
x_diff(1:4) = quatdiff(x_hat(1:4), x_eq(1:4));
x_diff_r = x_diff(2:end);
y = s.att.Cd * x_hat;
y_diff = r(2:4)-y(2:4);
result_y_int_increment = y_diff * s.att.Ts;
err = [ x_diff_r; y_int ];

% Calculate output
u_ctrl = K * err;
result_u = u_ctrl + u_eq;

end

function qd = quatdiff(ql, qr)
qd = quatmultiply(ql', quatconj(qr'))';
end