function [ result_x_hat ] = GenerateAltitudeObserver(s)
%GENERATE_ALTITUDE_OBSERVER
%   Code generator for the altitude observer. This function creates a sym that
%   can be used to replace tags with generated code in AltitudeCodegen.cpp.
%
%   @param  s
%           The result of GetParamsAndMatrices().
%
%   @return result_x_hat
%           Kalman state estimate, calculated from the syms
%           'vector__x_hat_copy', 'vector__u' and 'vector__y'.
%

% Round matrices
L = round(s.alt.kal.L, 8);

% Create syms
x_hat = sym('vector__x_hat_copy', [3, 1], 'real');
u     = sym('vector__u',          [1, 1], 'real');
y     = sym('vector__y',          [1, 1], 'real');

% Calculate estimate (Kalman)
prediction   = s.alt.Ad * x_hat + s.alt.Bd * u;  % A * x + B * u
dif          = y - s.alt.Cd * x_hat;
innovation   = L * dif;  % L * (y - C * x)
result_x_hat = prediction + innovation;

end