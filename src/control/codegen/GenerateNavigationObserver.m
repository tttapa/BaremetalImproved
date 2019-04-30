function result_x_hat = GenerateNavigationObserver(s)
%GenerateNavigationObserver - Code generator for the navigation observer
%
% Syntax: result_x_hat = GenerateNavigationObserver(s)
%
% `s` is the result of GetParamsAndMatrices that contains the Kalman matrix 
% `kal.L`

% Round matrices
L = round(s.nav.kal.L, 8);

% Create syms
x_hat         = sym('vector__x_hat_copy', [6, 1],  'real');
u             = sym('vector__u',          [2, 1],  'real');
att_x_hat     = sym('vector__att_x_hat',  [10, 1], 'real');
y_nav         = sym('vector__y',          [2, 1],  'real');
Ts            = sym('Ts', 'real');

y = [ att_x_hat(2:3); y_nav ]; %

% Use forward Euler to discretize the continuous system with a variable sample
% time
nx = size(s.nav.Aa, 1);
Ad = eye(nx) + s.nav.Aa * Ts;
Bd = s.nav.Ba * Ts;

% Calculate estimate
prediction   = Ad * x_hat + Bd * u;  % A * x + B * u
dif          = y - s.nav.Cd * x_hat;
innovation   = L * dif;  % L * (y - C * x)
result_x_hat = prediction + innovation;

end