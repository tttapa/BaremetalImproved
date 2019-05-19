% ../../MATLAB/quat_params.m

function p = quat_params(varargin)
%QUAT_PARAMS returns the nomimal parameters of the quadcopter in a
%structure.
%
%Parameters:
%
% SYMBOL   | UNITS | NAME
% --------------------------------------------------------------
% Vmax     | V     | maximum voltage
% Vmin     | V     | minimum voltage
% Nm       | -     | number of motors
% m        | kg    | total mass
% L        | m     | arm length
% rho      | kg/m3 | air density (nominal, at 15C, sea level)
% g        | m/s2  | gravitational acceleration
% ct       | -     | thrust coefficient
% cp       | -     | power coefficient
% mp       | kg    | propeller mass
% Dp       | m     | propeller diameter
% Kv       | rpm/V | motor speed constant
% tau_m    | s     | motor's time constant
% mr       | kg    | rotor mass (moving parts)
% mm       | kg    | motor total mass
% rr       | m     | rotor mass
% nh       | rps   | hovering frequency of rotation
% Ip       | kgm2  | propeller moment of inertia
% Im       | kgm2  | motor's moment of inertia
% Ixx      | kgm2  | moment of inertia about the x-axis
% Iyy      | kgm2  | moment of inertia about the y-axis
% Izz      | kgm2  | moment of inertia about the z-axis
% I        | kgm2  | diagonal matrix (Ixx, Iyy, Izz)
% k1       |       | parameter k1
% k2       | 1/s   | parameter k2
% k3       | (fun) | parameter k3
% k4       |       | parameter k4 (reaction wheel effect)
% kt       | ?     | eagle-control-slides.pdf Slide 162/186
% gamma_n  |       |
% gamma_u  |       |
% --------------------------------------------------------------
%
%Parameters gamma_n and gamma_u define the linearised n-s dynamics as
%follows
%
% dn/dt = k2 k1 u - k2 n
%     s = gamma_n n + gamma_u u
%
%Syntax:
% p = quat_params();
% p = quat_params(ops);
%
%Input arguments:
% ops       A structure where one may provide parameter values to be
%           overriden. 
%
%See also
% quat_dynamics


p.nominal = true;

% ---- Fixed Quantities ----

% general parameters
p.Vmax = 11.1;          % V ....... maximum voltage
p.Vmin = 0.1*p.Vmax;    % V ....... minimum voltage
p.Nm = 4;               % - ....... number of motors
p.m = 1.850;            % kg ...... total mass
p.L = 0.27;             % m ....... arm length
p.rho = 1.225;          % kg/m3 ... air density (nominal, at 15C, sea level)
p.g = 9.81;             % m/s2 .... gravitational acceleration

% propellers
p.ct = 0.1;             % - ....... thrust coefficient
p.cp = 0.04;            % - ....... power coefficient
p.mp = 20/1000;         % kg ...... propeller mass (20g)
p.Dp = 12*2.54/100;     % m ....... propeller diameter (12")

% motors
p.Kv = 700;             % rpm/V ... motor speed constant
p.tau_m = 35/1000;      % s ....... motor's time constant (35ms)
p.mr = 42/1000;         % kg ...... rotor mass (moving parts) (42g)
p.mm = 102/1000;        % kg ...... motor total mass (102g)
p.rr = 1.9/100;         % m ....... rotor radius (1.9cm)

% Moments of intertia
p.Ixx=0.0321;             % kgm2 .... Ixx moment of inertia
p.Iyy=0.0340;             % kgm2 .... Iyy moment of inertia
p.Izz=0.0575;             % kgm2 .... Izz moment of inertia

% ---- Override constants using options ----
if nargin > 0,
    o = varargin{1};
    if isstruct(o),
        pn = fieldnames(p);
        for i=1:length(pn),
            fname = pn(i);
            fname = fname{1};
            if isfield(o, fname),
                p.(fname) = o.(fname);
                p.nominal = false;
            end
        end
    end
end

% ---- Computed Quantities ----

p.nh = ...              % rps ... hovering n
    sqrt( (p.m*p.g)/(p.ct * p.rho * p.Dp^4 * p.Nm) );

% Very rough estimation of moments of inertia
p.Ip = p.mp*p.Dp^2/12;  % kgm2 .... propeller moment of inertia
p.Im = p.mr * p.rr^2;   % kgm2 .... rotor moment of inertia
p.I = diag([p.Ixx;
    p.Iyy;
    p.Izz]);            % kgm3 .... Inertia matrix

% model constants
p.k1= p.Kv*(p.Vmax-p.Vmin)/60;
p.k2 = 1/p.tau_m;
p.k3 = diag([2 * p.ct * p.rho * p.nh * p.Dp^4 * p.Nm * p.L / sqrt(2) / p.Ixx;
             2 * p.ct * p.rho * p.nh * p.Dp^4 * p.Nm * p.L / sqrt(2) / p.Iyy;
             2 * p.cp * p.rho * p.nh * p.Dp^5 * p.Nm / (2*pi*p.Izz) ]);
p.k4 = diag([0;
             0;
             2 * pi * p.Nm * (p.Im + p.Ip) / p.Izz]);


% Matrix Gamma_n
p.gamma_n = p.k3 -p.k2*p.k4;

% Matrix Gamma_u
p.gamma_u = diag([0,0,p.k4(3,3)*p.k2*p.k1]);

% OWN CONSTANTS
p.kt = p.Nm * p.ct * p.rho * p.Dp^4 / p.m;
p.uh = p.nh/p.k1;


