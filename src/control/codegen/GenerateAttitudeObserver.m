function [ result_x_hat ] = GenerateAttitudeObserver(s)
%GENERATE_ATTITUDE_OBSERVER
%   Code generator for the attitude observer. This function creates syms that
%   can be used to replace tags with generated code in AttitudeCodegen.cpp.
%
%   @param  s
%           The result of GetParamsAndMatrices().
%
%   @return result_x_hat
%           Kalman state estimate, calculated from the syms 'vector__x_hat',
%           'vector__u' and 'vector__y'.
%

% Round matrices
L = round(s.att.kal.L, 8);

% Create syms
x_hat = sym('vector__x_hat', [9, 1], 'real');
u     = sym('vector__u',     [3,  1], 'real');
y     = sym('vector__y',     [3,  1], 'real');

% Calculate estimate
prediction   = s.att.Ad * x_hat + a.att.Bd * u; % A * x + B * u
diff         = y - s.att.Cd * x_hat;
innovation   = L * diff;
result_x_hat = predication + innovation;

end
