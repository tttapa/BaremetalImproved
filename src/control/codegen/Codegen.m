clear

if ~exist('Output', 'dir'); mkdir('Output'); end
if ~exist('Output/src', 'dir'); mkdir('Output/src'); end
if ~exist('Output/ParamsAndMatrices', 'dir'); mkdir('Output/ParamsAndMatrices'); end

s = GetParamsAndMatrices();
p = quat_params();

%% 4 Configurations, config. 5 is reserved for callibration
% Config 1: current config
s1 = s;
s1.nav.lqr.Q = diag([3.0, 3.0, 0.9, 0.9, 0.015, 0.015]);
s1.nav.lqr.K = 200.0*eye(2);
s1.nav.lqr.K = -dlqr(s1.nav.Ad, s1.nav.Bd, s1.nav.lqr.Q, s1.nav.lqr.R);
s1.nav.lqi.I = 0.001*[0,-1;1,0];
s1.nav.lqi.max_integral = 10;
s1.nav.lqi.K = [s1.nav.lqr.K, s1.nav.lqi.I];

%s1.nav.lqr.Q = diag([0.01,0.01,0.10,0.10,0.002,0.002]);
%s1.nav.lqr.R = 30.0*eye(2);
%s1.nav.lqr.K = -dlqr(s1.nav.Ad, s1.nav.Bd, s1.nav.lqr.Q, s1.nav.lqr.R);
%s1.nav.lqi.I = 0.01 * [0, -1; 1, 0];
%s1.nav.lqi.max_integral = 10;
%s1.nav.lqi.K = [s1.nav.lqr.K, s1.nav.lqi.I];


% Config 2
s2 = s;
s2.nav.lqr.Q = diag([3.0, 3.0, 0.9, 0.9, 0.015, 0.015]);
s2.nav.lqr.K = 350.0*eye(2);
s2.nav.lqr.K = -dlqr(s2.nav.Ad, s2.nav.Bd, s2.nav.lqr.Q, s2.nav.lqr.R);
s2.nav.lqi.I = 0.001*[0,-1;1,0];
s2.nav.lqi.max_integral = 10;
s2.nav.lqi.K = [s2.nav.lqr.K, s2.nav.lqi.I];

%s2.nav.lqr.Q = diag([0.001,0.001,0.1,0.1,0.001,0.001]);
%s2.nav.lqr.R = 15.0*eye(2);
%s2.nav.lqr.K = -dlqr(s2.nav.Ad, s2.nav.Bd, s2.nav.lqr.Q, s2.nav.lqr.R);
%s2.nav.lqi.I = 0.01 * [0, -1; 1, 0];
%s2.nav.lqi.max_integral = 10;
%s2.nav.lqi.K = [s2.nav.lqr.K, s2.nav.lqi.I];


% Config 3
s3 = s;
s3.nav.lqr.Q = diag([1.0,1.0,0.3,0.3,0.001,0.001]);
s3.nav.lqr.R = 15.0*eye(2);
s3.nav.lqr.K = -dlqr(s3.nav.Ad, s3.nav.Bd, s3.nav.lqr.Q, s3.nav.lqr.R);
s3.nav.lqi.I = 0.01 * [0, -1; 1, 0];
s3.nav.lqi.max_integral = 10;
s3.nav.lqi.K = [s3.nav.lqr.K, s3.nav.lqi.I];


% Config 4
s4 = s;
s4.nav.lqr.Q = diag([0.01,0.01,0.3,0.3,0.001,0.001]);
s4.nav.lqr.R = 30.0*eye(2);
s4.nav.lqr.K = -dlqr(s4.nav.Ad, s4.nav.Bd, s4.nav.lqr.Q, s4.nav.lqr.R);
s4.nav.lqi.I = 0.01 * [0, -1; 1, 0];
s4.nav.lqi.max_integral = 10;
s4.nav.lqi.K = [s4.nav.lqr.K, s4.nav.lqi.I];

%s1 = s3;
%s2 = s3;
%s4 = s3;

configs = [s1, s2, s3, s4];


%% Attitude

% Output to file
template = fileread('Templates/AttitudeCodegenTemplate.cpp');
outputFid = fopen('Output/src/AttitudeCodegen.cpp', 'w');
    
fprintf(outputFid, '/* Automatically generated, using... \r\n');
for k = 1:length(configs)
    
    % Current controller
    s = configs(k);
    
    % Attitude controller
    [controlSignal, integralWindup] = GenerateAttitudeController(s);
    controlSignalElements = symbolicVectorExpressionToStrings(controlSignal); % Control output
    integralWindupElements = symbolicVectorExpressionToStrings(integralWindup); % Integral increment

    % Attitude observer
    [prediction, innovation, stateEstimate] = GenerateAttitudeObserver(s);
    PredictionElements = symbolicVectorExpressionToStrings(prediction); % Prediction (reduced)
    InnovationElements = symbolicVectorExpressionToStrings(innovation); % Innovation (reduced)
    stateEstimateElements = symbolicVectorExpressionToStrings(stateEstimate); % New state estimate x_hat (full)

    for i = 1:length(controlSignalElements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, controlSignalElements(i));
    end
    for i = 1:length(PredictionElements)
        tag = strcat('$c', num2str(k), '$p', num2str(i - 0));
        template = replace(template, tag, PredictionElements(i)); % reduced
    end
    for i = 1:length(InnovationElements)
        tag = strcat('$c', num2str(k), '$i', num2str(i - 0));
        template = replace(template, tag, InnovationElements(i)); % reduced
    end
    for i = 1:length(stateEstimate_elements)
        tag = strcat('$c', num2str(k), '$x', num2str(i - 1));
        template = replace(template, tag, stateEstimateElements(i));
    end
    tag = strcat('$c', num2str(k), '$maxWindup');
    template = replace(template, tag, num2str(s.att.lqi.max_integral));

    % Extend automatic signature with current controller
    fprintf(outputFid,  ' *\r\n');
    fprintf(outputFid, [' * Configuration ', char(num2str(k)), ': \r\n']);
    fprintf(outputFid,  ' * Q = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.att.lqr.Q, 1)) '\r\n'], s.att.lqr.Q');
    fprintf(outputFid,  ' * R = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.att.lqr.R, 1)) '\r\n'], s.att.lqr.R');

end
for i = 1:length(integralWindupElements)
    tag = strcat('$int', num2str(i - 1));
    template = replace(template, tag, integralWindupElements(i));
end
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);


%% Altitude 

% Output to file
template = fileread('Templates/AltitudeCodegenTemplate.cpp');
outputFid = fopen('Output/src/AltitudeCodegen.cpp', 'w');

fprintf(outputFid, '/* Automatically generated, using \r\n');
for k = 1:length(configs)
    
    % Current configuration
    s = configs(k);
    
    % Altitude controller
    [controlSignal, integralWindup] = GenerateAltitudeController(s);
    controlSignalElements = symbolicVectorExpressionToStrings(controlSignal); % Control output
    integralWindupElements = symbolicVectorExpressionToStrings(integralWindup); % Integral increment

    % Altitude observer
    stateEstimate = GenerateAltitudeObserver(s);
    stateEstimateElements = symbolicVectorExpressionToStrings(stateEstimate); % New state estimate x_hat

    for i = 1:length(controlSignalElements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, controlSignalElements(i));
    end
    for i = 1:length(stateEstimateElements)
        tag = strcat('$c', num2str(k), '$x', num2str(i - 1));
        template = replace(template, tag, stateEstimateElements(i));
    end
    tag = strcat('$c', num2str(k), '$maxWindup');
    template = replace(template, tag, num2str(s.alt.lqi.max_integral));
    
    % Extend automatic signature with current controller
    fprintf(outputFid,  ' *\r\n');
    fprintf(outputFid, [' * Configuration ', char(num2str(k)), ': \r\n']);
    fprintf(outputFid,  ' * Q = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.alt.lqr.Q, 1)) '\r\n'], s.alt.lqr.Q');
    fprintf(outputFid,  ' * R = %d\r\n', s.alt.lqr.R);
    fprintf(outputFid,  ' * I = %d\r\n', s.alt.lqi.I);
    
end
for i = 1:length(integralWindupElements)
    tag = strcat('$int', num2str(i - 1));
    template = replace(template, tag, integralWindupElements(i));
end
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);

%% Navigation

%Output to file
template = fileread('Templates/navigation-controller.template.c');
outputFid = fopen('Output/src/navigation-controller.c', 'w');

fprintf(outputFid, '/* Automatically generated, using \r\n');
for k = 1:length(configs)
    
    % Current controller
    s = configs(k);
    
    % Navigation controller
    [u, int] = GenerateNavigationController(s);
    u_elements = symbolicVectorExpressionToStrings(u); % Control output
    int_elements = symbolicVectorExpressionToStrings(int); % Integral increment

    % Navigation observer
    stateEstimate = GenerateNavigationObserver(s);
    stateEstimate_elements = symbolicVectorExpressionToStrings(stateEstimate); % New state estimate x_hat

    for i = 1:length(u_elements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, u_elements(i));
    end
    for i = 1:length(int_elements)
        tag = strcat('$c', num2str(k), '$i', num2str(i - 1));
        template = replace(template, tag, int_elements(i));
    end
    for i = 1:length(stateEstimate_elements)
        tag = strcat('$c', num2str(k), '$x', num2str(i - 1));
        template = replace(template, tag, stateEstimate_elements(i));
    end
    tag = strcat('$c', num2str(k), '$maxWindup');
    template = replace(template, tag, num2str(s.nav.lqi.max_integral));
    
    % Extend automatic signature with current controller
    fprintf(outputFid,  ' *\r\n');
    fprintf(outputFid, [' * Configuration ', char(num2str(k)), ': \r\n']);
    fprintf(outputFid,  ' * Q = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.nav.lqr.Q, 1)) '\r\n'], s.nav.lqr.Q');
    fprintf(outputFid,  ' * R = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.nav.lqr.R, 1)) '\r\n'], s.nav.lqr.R');
    fprintf(outputFid,  ' * I = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.nav.lqi.I, 1)) '\r\n'], s.nav.lqi.I');
    
end
for i = 1:length(int_elements)
    tag = strcat('$int', num2str(i - 1));
    template = replace(template, tag, int_elements(i));
end
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);

ExportParamsAndMatrices('Output/ParamsAndMatrices');

%% Symbolic expression to string
function textComponents = symbolicVectorExpressionToStrings(u)
u = vpa(u, 17);
u_str = char(u);
u_str = replace(u_str, 'matrix([[', '');
u_str = replace(u_str, ']])', '');

u_str = replace(u_str, '], [', ',');

regex = 'vector__(\w+?)(\d+)';
repl = '$1[$2 - 1]';
u_str = regexprep(u_str, regex, repl);
regex = 'matrix__(\w+?)(\d+)_(\d+)';
repl = '$1[$2 - 1][$3 - 1]';
u_str = regexprep(u_str, regex, repl);

textComponents = strsplit(u_str, ',');
end
