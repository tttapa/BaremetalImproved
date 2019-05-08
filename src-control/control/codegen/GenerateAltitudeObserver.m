function result_x_hat = GenerateAltitudeObserver(s)
%GenerateAltitudeObserver - Code generator for the altitude observer
%
% Syntax: result_x_hat = GenerateAltitudeObserver(s)
%
% `s` is the result of GetParamsAndMatrices that contains the Kalman matrix 
% `L_kal`

% Round matrices
L = round(s.alt.kal.L, 8);

% Create syms
x_hat = sym('vector__x_hat_copy', [3, 1], 'real');
u     = sym('vector__u',          [1, 1], 'real');
y     = sym('vector__y',          [1, 1], 'real');

% Calculate estimate
prediction   = s.alt.Ad * x_hat + s.alt.Bd * u;  % A * x + B * u
dif          = y - s.alt.Cd * x_hat;
innovation   = L * dif;  % L * (y - C * x)
result_x_hat = prediction + innovation;

end