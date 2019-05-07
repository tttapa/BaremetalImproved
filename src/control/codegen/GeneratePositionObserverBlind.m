function result_x_hat = GeneratePositionObserverBlind(s)
%GenerateNavigationObserver - Code generator for the navigation observer
%
% Syntax: result_x_hat = GenerateNavigationObserver(s)
%
% `s` is the result of GetParamsAndMatrices that contains the system
% matrices for the blind position observer.

% Create syms
x_hat         = sym('vector__x_hat_blind_copy', [4, 1],  'real');
u             = sym('vector__u',                [2, 1],  'real');


% Calculate estimate
Ad = s.pos.ABlindd;
Bd = s.pos.BBlindd;
result_x_hat   = Ad * x_hat + Bd * u;  % A * x + B * u

end