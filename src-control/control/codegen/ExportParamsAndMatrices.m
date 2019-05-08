% ../ParamsAndMatrices/ExportParamsAndMatrices.m

function ExportParamsAndMatrices(output_dir)

s = GetParamsAndMatrices();

if (nargin == 0)
    if ~exist('Output', 'dir'); mkdir('Output'); end
    cd('Output');
else
    cd(output_dir);
end

%% Attitude

exportdouble(s.att.Ts, 'attitude/Ts');

exportmatrix(s.att.Aa, 'attitude/Aa');
exportmatrix(s.att.Ba, 'attitude/Ba');
exportmatrix(s.att.Ca, 'attitude/Ca');
exportmatrix(s.att.Da, 'attitude/Da');

exportmatrix(s.att.Ad, 'attitude/Ad');
exportmatrix(s.att.Bd, 'attitude/Bd');
exportmatrix(s.att.Cd, 'attitude/Cd');
exportmatrix(s.att.Dd, 'attitude/Dd');

exportmatrix(s.p.gamma_n, 'attitude/gamma_n');
exportmatrix(s.p.gamma_u, 'attitude/gamma_u');

exportmatrix(s.att.lqr.Q, 'attitude/lqr/Q');
exportmatrix(s.att.lqr.R, 'attitude/lqr/R');
exportmatrix(s.att.lqr.K, 'attitude/lqr/K');

exportmatrix(s.att.kal.Q, 'attitude/kal/Q');
exportmatrix(s.att.kal.R, 'attitude/kal/R');
exportmatrix(s.att.kal.L, 'attitude/kal/L');

%% Altitude

exportdouble(s.alt.Ts, 'altitude/Ts');

exportmatrix(s.alt.Aa, 'altitude/Aa');
exportmatrix(s.alt.Ba, 'altitude/Ba');
exportmatrix(s.alt.Ca, 'altitude/Ca');
exportmatrix(s.alt.Da, 'altitude/Da');

exportmatrix(s.alt.Ad, 'altitude/Ad');
exportmatrix(s.alt.Bd, 'altitude/Bd');
exportmatrix(s.alt.Cd, 'altitude/Cd');
exportmatrix(s.alt.Dd, 'altitude/Dd');

exportmatrix(s.alt.lqr.Q, 'altitude/lqr/Q');
exportmatrix(s.alt.lqr.R, 'altitude/lqr/R');
exportmatrix(s.alt.lqr.K, 'altitude/lqr/K');
exportmatrix(s.alt.lqi.I, 'altitude/lqi/I');
exportmatrix(s.alt.lqi.K, 'altitude/lqi/K');
exportdouble(s.alt.lqi.max_integral, 'altitude/lqi/max_integral');

exportmatrix(s.alt.kal.Q, 'altitude/kal/Q');
exportmatrix(s.alt.kal.R, 'altitude/kal/R');
exportmatrix(s.alt.kal.L, 'altitude/kal/L');

%% General

I_inv = inv(s.p.I);

exportmatrix(s.p.I, 'I');
exportmatrix(I_inv, 'I_inv');

exportdouble(s.p.k1, 'k1');
exportdouble(s.p.k2, 'k2');

exportdouble(s.p.m, 'm');
exportdouble(s.p.ct, 'ct');
exportdouble(s.p.Dp, 'Dp');

exportmatrix([1,2,3; 4,5,6], 'test')
exportdouble(pi, 'test');

end

%% Export functions

function exportmatrix(M, name)
    folder = fileparts(name);
    if ~(string(folder) == '') && ~exist(folder, 'dir'); mkdir(folder); end
    filename = strcat(name, '.matrix');
    fileID = fopen(filename,'w');
    [r, c] = size(M);
    fwrite(fileID, r, 'uint8');
    fwrite(fileID, c, 'uint8');
    fwrite(fileID,M','double');
    fclose(fileID);
end

function exportdouble(d, name)
    folder = fileparts(name);
    if ~(string(folder) == '') && ~exist(folder, 'dir'); mkdir(folder); end
    filename = strcat(name, '.double');
    fileID = fopen(filename,'w');
    fwrite(fileID,d,'double');
    fclose(fileID);
end
