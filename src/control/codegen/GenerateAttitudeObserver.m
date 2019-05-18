function [ result_prediction_r, result_innovation_r, result_x_hat ] = GenerateAttitudeObserver(s)
%GENERATE_ATTITUDE_OBSERVER
%   Code generator for the attitude observer. This function creates syms that
%   can be used to replace tags with generated code in AttitudeCodegen.cpp.
%
%   @param  s
%           The result of GetParamsAndMatrices().
%
%   @return result_prediction_r
%           Kalman state prediction Ax+Bu, calculated from the syms
%           'vector__x_hat' and 'vector__u'.
%   @return result_innovation_r
%           Kalman state innovation L*(y-Cx), calculated from the syms
%           'vector__x_hat' and 'vector__y'.
%   @return result_x_hat
%           Kalman state estimate, calculated from the syms 'vector__x_hat',
%           'vector__u' and 'vector__y'.
%

% Ignore very small values
L = round(s.att.kal.L, 8);

% Create syms
x_hat = sym('vector__x_hat', [10, 1], 'real');
u     = sym('vector__u',     [3,  1], 'real');
y     = sym('vector__y',     [7,  1], 'real');

% Calculate estimate
x_hat_red           = x_hat(2:end);
result_prediction_r = s.att.Ad_r * x_hat_red + s.att.Bd_r * u;  % A * x + B * u

cx                  = s.att.Cd * x_hat;
dif                 = y - cx;
dif(1:4)            = quatdiff(y(1:4), cx(1:4));
dif_r               = dif(2:end);
result_innovation_r = L * dif_r;  % L * (y - C * x)

sym_prediction    = sym('vector__prediction', [10, 1], 'real');
sym_innovation    = sym('vector__innovation', [10, 1], 'real');

result_x_hat        = sym_prediction + sym_innovation;
result_x_hat(1:4)   = quatmult(sym_prediction(1:4), sym_innovation(1:4));

end

function qd = quatdiff(ql, qr)
qd = quatmultiply(ql', quatconj(qr'))';
end

function qm = quatmult(ql, qr)
qm = quatmultiply(ql', qr')';
end
