% in .../.../MATLAB/GetParamsAndMatrices.m

function s = GetParamsAndMatrices()

p = quat_params();
s.p = p;
method = 'zoh';

%% Attitude

% Linear system

s.att.fs = 238;
s.att.Ts = 1 / s.att.fs;

s.att.Aa = [ zeros(1, 10);
             zeros(3, 4), 0.5 * eye(3), zeros(3, 3);
             zeros(3, 7), p.gamma_n;
             zeros(3, 7), -p.k2 * eye(3) ];
s.att.Ba = [ zeros(4, 3);
             p.gamma_u;
             p.k2 * p.k1 * eye(3) ];
s.att.Ca = [ eye(7), zeros(7, 3) ];
s.att.Da =   zeros(7, 3);

continuousSys = ss(s.att.Aa, s.att.Ba, s.att.Ca, s.att.Da);
discreteSys = c2d(continuousSys, s.att.Ts, method);

s.att.Ad = discreteSys.A;
s.att.Bd = discreteSys.B;
s.att.Cd = discreteSys.C;
s.att.Dd = discreteSys.D;

s.att.Ad_r = s.att.Ad(2:10, 2:10);
s.att.Bd_r = s.att.Bd(2:10, 1:3);
s.att.Cd_r = s.att.Cd(2:7, 2:10);
s.att.Dd_r = s.att.Dd(2:7, 1:3);

% LQR

s.att.lqr.W  = [ s.att.Ad - eye(10),  s.att.Bd;
                 s.att.Cd,            s.att.Dd ];
s.att.lqr.OI = [ zeros(10, 4);
                 eye(4);
                 zeros(3, 4) ];
s.att.lqr.G = s.att.lqr.W \ s.att.lqr.OI;

% s.att.lqr.Q = diag([ 139.6245112700232, 139.6245112700232, 240.2811761590895, ...
%     0.1505204155597211, 0.1505204155597211, 0.0409919487616804, ...
%     9.976475759487083e-11, 9.976475759487083e-11, 9.976475759487083e-11]);
% s.att.lqr.R = diag([1, 1, 1.001966068300933]);

s.att.lqr.Q = diag([139.6245112700232,139.6245112700232,15.2811761590895,...
    1.1505204155597211,1.1505204155597211,0.1209919487616804,...
    9.976475759487083e-08,9.976475759487083e-08,9.976475759487083e-09]);

% s.att.lqr.R = 8*diag([1,1,1]);
% A bit more aggressive: 6.0
% s.att.lqr.R = 6.0*diag([1,1,1]);
% New battery (x4)
s.att.lqr.R = 24.0*diag([1,1,1]);
s.att.lqr.K = -dlqr(s.att.Ad_r, s.att.Bd_r, s.att.lqr.Q, s.att.lqr.R);
s.att.lqi.I = diag([0.8, 0.8, 0]);  % For anti bias drift
s.att.lqi.max_integral = 10;
s.att.lqi.K = [s.att.lqr.K, s.att.lqi.I];


% Kalman

% x_k+1   = A x_k + B u_k + G w_k
%   G_kal = [ eye(9), B ]
%   w_k   = [ dx_k
%             du_k ]

s.att.kal.varDynX = 10 * ones(1, 9);
s.att.kal.varDynU = 0.1 * ones(1, 3);
s.att.kal.Q = [ s.att.kal.varDynX, s.att.kal.varDynU ];
s.att.kal.G = [ eye(9), s.att.Bd_r ];
% qVar =  eul2quat(pi / 180.0 * ones(1, 3)).^2; % one degree of standard deviation
% omegaVar = 0.05.^2 * ones(1, 3);
s.att.kal.R = 0.1 * ones(1, 6);

s.att.kal.L = dlqe(s.att.Ad_r, s.att.kal.G, s.att.Cd_r, diag(s.att.kal.Q), diag(s.att.kal.R));

%% Altitude

% s.alt.fs = s.att.fs / 24;
s.alt.fs = 20;  % half 19, half 21 on drone
s.alt.Ts = 1 / s.alt.fs;

s.alt.kt = p.kt; % p.ct * p.rho * p.Dp^4 * p.Nm;
a = 2 * p.nh * p.kt; % s.alt.kt;

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

% LQI

s.alt.lqr.W  = [ s.alt.Ad - eye(3),  s.alt.Bd;
                 s.alt.Cd,           s.alt.Dd ];
s.alt.lqr.OI = [ zeros(3, 1);
                 eye(1) ];
s.alt.lqr.G = s.alt.lqr.W \ s.alt.lqr.OI;


%s.alt.lqr.Q = diag([ 1, 100, 1 ]);
%s.alt.lqr.R = diag([ 1 ]);
%s.alt.lqi.Q = blkdiag(s.alt.lqr.Q, 0.5);
%s.alt.lqi.R = s.alt.lqr.R;
%s.alt.lqr.K = -dlqr(s.alt.Ad, s.alt.Bd, s.alt.lqr.Q, s.alt.lqr.R);
%s.alt.lqi.K = dlqi(s.alt.Ad, s.alt.Bd, s.alt.Cd, s.alt.Dd, s.alt.lqi.Q, s.alt.lqi.R, s.alt.Ts);
% TODO
%s.alt.lqi.K = [0.0, 0.9, 0.5, -0.01];

% Altitude LQR from simulation
%s.alt.lqr.Q = diag([ 0, 0, 0 ]);
%s.alt.lqr.R = diag([ 0 ]);
%s.alt.lqr.K = [0.0, 0.9, 0.5];
%s.alt.lqi.I =  0.1;
%s.alt.lqi.K = [s.alt.lqr.K, s.alt.lqi.I];
%s.alt.lqi.max_integral = 10;

% Best LQR from 2nd try simulations
s.alt.lqr.Q = diag([1e-6, 0.8, 0.5]);
s.alt.lqr.R = diag([30]);
s.alt.lqr.K = -dlqr(s.alt.Ad, s.alt.Bd, s.alt.lqr.Q, s.alt.lqr.R);
s.alt.lqi.I = 0.010;
s.alt.lqi.K = [s.alt.lqr.K, s.alt.lqi.I];
s.alt.lqi.max_integral = 10;


% Kalman

% x_k+1   = A x_k + B u_k + G w_k
%   G_kal = [ eye(9), B ]
%   w_k   = [ dx_k
%             du_k ]


%% BEFORE THIS WEEKEND
%s.alt.kal.varDynX = 10 * ones(1, 3);
%s.alt.kal.varDynU = 0.01 * ones(1, 1);
%s.alt.kal.Q = [ s.alt.kal.varDynX, s.alt.kal.varDynU ];
%s.alt.kal.G = [ eye(3) ];
%s.alt.kal.G = [ eye(3), s.alt.Bd ];
%s.alt.kal.R = [ 0.03 ];

%% CURRENT
s.alt.kal.varDynX = [3, 3, 25]; % * ones(1, 3);
s.alt.kal.varDynU = 0.0015 * ones(1, 1);
s.alt.kal.Q = [ s.alt.kal.varDynX, s.alt.kal.varDynU ];
s.alt.kal.G = [ eye(3), s.alt.Bd ];
s.alt.kal.R = [ 0.0008 ];
s.alt.kal.L = dlqe(s.alt.Ad, s.alt.kal.G, s.alt.Cd, diag(s.alt.kal.Q), diag(s.alt.kal.R));


%% ORIGINAL KALMAN FROM DECEMBER
%s.alt.kal.varDynX = 0.1 * ones(1, 3);
%s.alt.kal.varDynU = 0.1 * ones(1, 1);
%s.alt.kal.Q = [ s.alt.kal.varDynX, s.alt.kal.varDynU ];
%s.alt.kal.G = [ eye(3), s.alt.Bd ];
%s.alt.kal.R = [ 1 ];
%s.alt.kal.L = dlqe(s.alt.Ad, s.alt.kal.G, s.alt.Cd, diag(s.alt.kal.Q), diag(s.alt.kal.R));


%% Position
%TODO fs nog invullen
% TODO: lambda?
% s.nav.lambda = 1.2; % TODO: 3.5?
s.nav.lambda = 3.0;
s.nav.fs = 8.5;
s.nav.Ts = 1.0 / s.nav.fs;

s.nav.Aa = [-s.nav.lambda,      0,         0, 0, 0, 0;
            0,            -s.nav.lambda,   0, 0, 0, 0;
            0,                  0,         0, 0, 1, 0;
            0,                  0,         0, 0, 0, 1;
            0,                2*p.g,       0, 0, 0, 0;
            -2*p.g,             0,         0, 0, 0, 0];
        
s.nav.Ba = [s.nav.lambda,       0;
            0,            s.nav.lambda;
            0,                  0;
            0,                  0;
            0,                  0;
            0,                  0];
        
s.nav.Ca = [eye(4),zeros(4, 2)];
s.nav.Da = zeros(4, 2);

continuousSys = ss(s.nav.Aa, s.nav.Ba, s.nav.Ca, s.nav.Da);
discreteSys = c2d(continuousSys, s.nav.Ts, method);

s.nav.Ad = discreteSys.A;
s.nav.Bd = discreteSys.B;
s.nav.Cd = discreteSys.C;
s.nav.Dd = discreteSys.D;



% Blind position
s.nav.ABlinda = [0, 0, 1, 0;
                 0, 0, 0, 1;
                 0, 0, 0, 0;
                 0, 0, 0, 0];

s.nav.BBlinda = [0, 0;
                 0, 0;
                 0, 2*p.g;
                 -2*p.g, 0];

s.nav.CBlinda = [];
s.nav.DBlinda = [];

continuousSys = ss(s.nav.ABlinda, s.nav.BBlinda, s.nav.CBlinda, s.nav.DBlinda);
discreteSys = c2d(continuousSys, s.att.Ts, method);

s.nav.ABlindd = discreteSys.A;
s.nav.BBlindd = discreteSys.B;
s.nav.CBlindd = discreteSys.C;
s.nav.DBlindd = discreteSys.D;

% LQI

s.nav.lqr.W = [ s.nav.Ad - eye(6), s.nav.Bd;
                s.nav.Cd,          s.nav.Dd ];
s.nav.lqr.OI = [zeros(6, 4);
                  eye(4)  ];
s.nav.lqr.G = s.nav.lqr.W \ s.nav.lqr.OI;


% TODO: lqi???
%s.nav.lqi.Q = blkdiag(s.nav.lqr.Q, 0.5, 0.5); %Ik heb 0.5 overgenomen van altitude, klopt dit?
%s.nav.lqi.R = s.nav.lqr.R;
%s.nav.lqi.K = dlqi(s.nav.Ad, s.nav.Bd, s.nav.Cd, s.nav.Dd, s.nav.lqi.Q, s.nav.lqi.R, s.nav.Ts);

% Navigation LQR from simulation
%s.nav.lqr.Q = diag([0.001,0.001,0.05,0.05,0.001,0.001]);
%s.nav.lqr.R = eye(2);
%s.nav.lqr.K = -dlqr(s.nav.Ad, s.nav.Bd, s.nav.lqr.Q, s.nav.lqr.R);
%s.nav.lqi.I = 0.01 * [0, -1; 1, 0];
%s.nav.lqi.max_integral = 10;
%s.nav.lqi.K = [s.nav.lqr.K, s.nav.lqi.I];

% Navigation LQRs from simulation 2.0 (w/noise & forward-euler KF)
s.nav.lqr.Q = diag([0.01,0.01,1.6,1.6,0.4,0.4]);
s.nav.lqr.R = 30.0*eye(2);
s.nav.lqr.K = -dlqr(s.nav.Ad, s.nav.Bd, s.nav.lqr.Q, s.nav.lqr.R);
s.nav.lqi.I = 0.01 * [0, -1; 1, 0];
s.nav.lqi.max_integral = 10;
s.nav.lqi.K = [s.nav.lqr.K, s.nav.lqi.I];


% Kalman
s.nav.kal.Q = [1e-4, 1e-4, 0.1, 0.1, 0.001, 0.001];
s.nav.kal.R = [1e-9, 1e-9,   1,   1];
s.nav.kal.G = eye(6);
s.nav.kal.L = dlqe(s.nav.Ad, s.nav.kal.G, s.nav.Cd, diag(s.nav.kal.Q), diag(s.nav.kal.R));


end