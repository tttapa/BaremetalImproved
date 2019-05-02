function [result_prediction_r, result_innovation_r, result_x_hat] = GenerateAttitudeObserver(s)
%GenerateAttitudeObserver - Code generator for the attitude observer
%
% Syntax: [result_prediction, result_innovation, result_x_hat] = GenerateAttitudeObserver(s)
%
% `s` is the result of GetParamsAndMatrices that contains the Kalman matrix 
% `L_kal`

% Round matrices
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
