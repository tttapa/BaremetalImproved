function [ s ] = GetParamsAndMatrices()
%GET_PARAMS_AND_MATRICES
%   Create a struct containing the attitude control system, the altitude control
%   system and the position control system. Each of these contains the linear
%   system, a LQR controller with integral action and an observer. Attitude and
%   altitude use a Kalman filter for the observer, while position calculates the
%   estimate using discrete derivation and jump rejection.
%
%   @return s
%           Struct containing the attitude control system (s.att), the altitude
%           control system (s.alt) and the position control system (s.pos).
%

p = quat_params();
s.p = p;
method = 'zoh';

%% Attitude (LQR + integral action with Kalman estimate)

% Linear system
s.att.fs = 952; % TODO: sync this with BareMetal
s.att.Ts = 1 / s.att.fs;

s.att.Aa = [ zeros(3), eye(3)  , zeros(3);
             zeros(3), zeros(3), p.gamma_n;
             zeros(3), zeros(3), -p.k2*eye(3) ];
s.att.Ba = [ zeros(3); p.gamma_u; p.k2 * p.k1 * eye(3) ];
s.att.Ca = [ eye(3); zeros(3, 6) ];
s.att.Da = zeros(6,3);
continuousSys = ss(s.att.Aa, s.att.Ba, s.att.Ca, s.att.Da);
discreteSys = c2d(continuousSys, s.att.Ts, method);
s.att.Ad = discreteSys.A;
s.att.Bd = discreteSys.B;
s.att.Cd = discreteSys.C;
s.att.Dd = discreteSys.D;

% LQR with integral action
s.att.lqr.W  = [ s.att.Ad - eye(9), s.att.Bd;
                 s.att.Cd,          s.att.Dd ];
s.att.lqr.OI = [ zeros(9, 3); eye(3)];
s.att.lqr.G = s.att.lqr.W \ s.att.lqr.OI;

s.att.lqr.Q = diag([139.6245112700232/2.0,139.6245112700232/2.0,15.2811761590895/2.0,...
    1.1505204155597211,1.1505204155597211,0.1209919487616804,...
    9.976475759487083e-08,9.976475759487083e-08,9.976475759487083e-09]);
s.att.lqr.R = 24.0*diag([1,1,1]);
s.att.lqr.K = -dlqr(s.att.Ad, s.att.Bd, s.att.lqr.Q, s.att.lqr.R);
s.att.lqi.I = diag([0.1, 0.1, 0.01]);  % For anti bias drift
s.att.lqi.max_integral = 10;
s.att.lqi.K = [s.att.lqr.K, s.att.lqi.I];

% Kalman:
%   x_k+1   = A x_k + B u_k + G w_k
%   G_kal = [ eye(9), B ]
%   w_k   = [ dx_k
%             du_k ]
s.att.kal.varDynX = 10 * ones(1, 9);
s.att.kal.varDynU = 0.1 * ones(1, 3);
s.att.kal.Q = [ s.att.kal.varDynX, s.att.kal.varDynU ];
s.att.kal.G = [ eye(9), s.att.Bd ];
s.att.kal.R = 0.05 * ones(1, 3);
s.att.kal.L = dlqe(s.att.Ad, s.att.kal.G, s.att.Cd, diag(s.att.kal.Q), diag(s.att.kal.R));


%% Altitude (LQR + integral action with Kalman estimate)

% Linear system
s.alt.fs = 20;
s.alt.Ts = 1 / s.alt.fs;

s.alt.kt = p.kt;
a = 2 * p.nh * p.kt;
s.alt.Aa = [ -p.k2  0  0;
               0    0  1;
               a    0  0 ];
s.alt.Ba = [ p.k1 * p.k2; 
                  0; 
                  0 ];
s.alt.Ca = [ 0 1 0 ];
s.alt.Da =   zeros(1, 1);
continuousSys = ss(s.alt.Aa, s.alt.Ba, s.alt.Ca, s.alt.Da);
discreteSys = c2d(continuousSys, s.alt.Ts, method);
s.alt.Ad = discreteSys.A;
s.alt.Bd = discreteSys.B;
s.alt.Cd = discreteSys.C;
s.alt.Dd = discreteSys.D;

% LQR with integral action
s.alt.lqr.W  = [ s.alt.Ad - eye(3),  s.alt.Bd;
                 s.alt.Cd,           s.alt.Dd ];
s.alt.lqr.OI = [ zeros(3, 1);
                 eye(1) ];
s.alt.lqr.G = s.alt.lqr.W \ s.alt.lqr.OI;

% TODO: altitude tuning
% Best LQR from 2nd try simulations
s.alt.lqr.Q = diag([1e-6, 0.8, 0.5]);
s.alt.lqr.R = diag([30]);
s.alt.lqr.K = -dlqr(s.alt.Ad, s.alt.Bd, s.alt.lqr.Q, s.alt.lqr.R);
s.alt.lqi.I = 0.010;
s.alt.lqi.K = [s.alt.lqr.K, s.alt.lqi.I];
s.alt.lqi.max_integral = 10;

% TODO: altitude tuning
% Kalman
% x_k+1   = A x_k + B u_k + G w_k
%   G_kal = [ eye(9), B ]
%   w_k   = [ dx_k
%             du_k ]
s.alt.kal.varDynX = [ 3, 3, 25 ];
s.alt.kal.varDynU = [ 0.0015 ];
s.alt.kal.Q = [ s.alt.kal.varDynX, s.alt.kal.varDynU ];
s.alt.kal.G = [ eye(3), s.alt.Bd ];
s.alt.kal.R = [ 0.0008 ];
s.alt.kal.L = dlqe(s.alt.Ad, s.alt.kal.G, s.alt.Cd, diag(s.alt.kal.Q), diag(s.alt.kal.R));


%% Position (LQR + integral action with *** estimate)
% ***   q1 - from attitude estimate
%       q2 - from attitude estimate
%       x  - from IMP (with tilt correction)
%       y  - from IMP (with tilt correction)
%       vx - (x_new - x_old) / s.pos.Ts (with jump rejection)
%       vy - (y_new - y_old) / s.pos.Ts (with jump rejection)
%

% Linear system
s.pos.lambda = 3.5; % TODO: lambda = 3.5 should be good enough for LQR I think
s.pos.fs = 60.0; % TODO: position average FPS?
s.pos.Ts = 1.0 / s.pos.fs;

s.pos.Aa = [-s.pos.lambda,      0,         0, 0, 0, 0;
            0,            -s.pos.lambda,   0, 0, 0, 0;
            0,                  0,         0, 0, 1, 0;
            0,                  0,         0, 0, 0, 1;
            0,                2*p.g,       0, 0, 0, 0;
            -2*p.g,             0,         0, 0, 0, 0];
        
s.pos.Ba = [s.pos.lambda,       0;
            0,            s.pos.lambda;
            0,                  0;
            0,                  0;
            0,                  0;
            0,                  0];
s.pos.Ca = [eye(4),zeros(4, 2)];
s.pos.Da = zeros(4, 2);
continuousSys = ss(s.pos.Aa, s.pos.Ba, s.pos.Ca, s.pos.Da);
discreteSys = c2d(continuousSys, s.pos.Ts, method);

s.pos.Ad = discreteSys.A;
s.pos.Bd = discreteSys.B;
s.pos.Cd = discreteSys.C;
s.pos.Dd = discreteSys.D;

% LQR with integral action
s.pos.lqr.W = [ s.pos.Ad - eye(6), s.pos.Bd;
                s.pos.Cd,          s.pos.Dd ];
s.pos.lqr.OI = [zeros(6, 4);
                  eye(4)  ];
s.pos.lqr.G = s.pos.lqr.W \ s.pos.lqr.OI;

% TODO: choose best LQR from configurations
s.pos.lqr.Q = diag([3.0, 3.0, 0.9, 0.9, 0.015, 0.015]);
s.pos.lqr.R = 200.0*eye(2);
s.pos.lqr.K = -dlqr(s.pos.Ad, s.pos.Bd, s.pos.lqr.Q, s.pos.lqr.R);
s.pos.lqi.I = 0.001*[0,-1;1,0];
s.pos.lqi.max_integral = 10;
s.pos.lqi.K = [s.pos.lqr.K, s.pos.lqi.I];


%% Position (blind)

% Linear system (reduced)
s.posBlind.Ts = s.att.Ts;
s.posBlind.Aa = [0, 0, 1, 0;
                 0, 0, 0, 1;
                 0, 0, 0, 0;
                 0, 0, 0, 0];
s.posBlind.Ba = [0, 0;
                 0, 0;
                 0, 2*p.g;
                 -2*p.g, 0];
s.posBlind.Ca = [];
s.posBlind.Da = [];
continuousSys = ss(s.posBlind.Aa, s.posBlind.Ba, s.posBlind.Ca, s.posBlind.Da);
discreteSys = c2d(continuousSys, s.posBlind.Ts, method);
s.posBlind.Ad = discreteSys.A;
s.posBlind.Bd = discreteSys.B;
s.posBlind.Cd = discreteSys.C;
s.posBlind.Dd = discreteSys.D;

% Linear system (full)
continuousSys = ss(s.pos.Aa, s.pos.Ba, s.pos.Ca, s.pos.Da);
discreteSys = c2d(continuousSys, s.posBlind.Ts, method);
s.posBlind.Adfull = discreteSys.A;
s.posBlind.Bdfull = discreteSys.B;
s.posBlind.Cdfull = discreteSys.C;
s.posBlind.Ddfull = discreteSys.D;

% LQR with integral action
s.posBlind.lqr.W = [ s.posBlind.Adfull - eye(6), s.posBlind.Bdfull;
                     s.posBlind.Cdfull,          s.posBlind.Ddfull ];
s.posBlind.lqr.OI = [zeros(6, 4);
                     eye(4)  ];
s.posBlind.lqr.G = s.posBlind.lqr.W \ s.posBlind.lqr.OI;

% TODO: choose best LQR from configurations
s.posBlind.lqr.Q = s.pos.lqr.Q;
s.posBlind.lqr.R = s.pos.lqr.R;
s.posBlind.lqr.K = -dlqr(s.posBlind.Adfull, s.posBlind.Bdfull, ...
                         s.posBlind.lqr.Q,  s.posBlind.lqr.R);
s.posBlind.lqi.I = s.pos.lqi.I;
s.posBlind.lqi.max_integral = s.pos.lqi.max_integral;
s.posBlind.lqi.K = [s.posBlind.lqr.K, s.posBlind.lqi.I];

end