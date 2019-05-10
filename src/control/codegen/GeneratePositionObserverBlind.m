function [ result_x_hat ] = GeneratePositionObserverBlind(s)
%GENERATE_POSITION_OBSERVER_BLIND
%   Code generator for the blind position observer. This function creates a sym
%   that can be used to replace tags with generated code in PositionCodegen.cpp.
%
%   @param  s
%           The result of GetParamsAndMatrices().
%
%   @return result_x_hat
%           Blind system state estimate Ax+Bu, calculated from the syms
%           'vector__x_hat_blind_copy' and 'vector__u'.
%

% Create syms
x_hat         = sym('vector__x_hat_blind_copy', [4, 1],  'real');
u             = sym('vector__u',                [2, 1],  'real');

% Calculate estimate
Ad = s.posBlind.Ad;
Bd = s.posBlind.Bd;
result_x_hat   = Ad * x_hat + Bd * u;  % A * x + B * u

end