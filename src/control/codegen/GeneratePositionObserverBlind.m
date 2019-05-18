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
%           'vector__x_hat' and 'vector__u'.
%

% Create syms
x_hat = sym('vector__x_hat', [6, 1], 'real');
u     = sym('vector__u',     [2, 1], 'real');

% Calculate system estimate (uncontrollable)
result_x_hat = s.posBlind.Ad * x_hat + s.posBlind.Bd * u; % A * x + B * u

end