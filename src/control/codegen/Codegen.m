function Codegen()
%CODEGEN
%   First, the function will create the attitude control system, altitude
%   control system and position control system. From that, syms will be used
%   to simplify each system's control signal calculation and estimate
%   calculation. Next, these syms will be replaced by the struct.member
%   representation used in BaremetalImproved. Finally the tags in the templates
%   will be replaced by this generated code in order to create the files
%   AttitudeCodegen.cpp, AltitudeCodegen.cpp and PositionCodegen.cpp.
%

clear

[scriptfolder, ~] = fileparts(mfilename('fullpath'));
addpath(scriptfolder);
inputfolder = fullfile(scriptfolder, '..', 'templates');

if ~exist('src-generated', 'dir'); mkdir('src-generated'); end
if ~exist('ParamsAndMatrices-generated', 'dir'); mkdir('ParamsAndMatrices-generated'); end

s = GetParamsAndMatrices();
p = quat_params();

%% 4 Configurations, configuration 5 is reserved for calibration

% Configuration 1
s1 = s;
s1.pos.lqr.Q = diag([3.0, 3.0, 0.9, 0.9, 0.015, 0.015]);
s1.pos.lqr.R = 30.0*eye(2);
s1.pos.lqr.K = 200.0*eye(2);
s1.pos.lqr.K = -dlqr(s1.pos.Ad, s1.pos.Bd, s1.pos.lqr.Q, s1.pos.lqr.R);
s1.pos.lqi.I = 0.001*[0,-1;1,0];
s1.pos.lqi.max_integral = 10;
s1.pos.lqi.K = [s1.pos.lqr.K, s1.pos.lqi.I];


% Configuration 2
s2 = s;
s2.pos.lqr.Q = diag([3.0, 3.0, 0.9, 0.9, 0.015, 0.015]);
s2.pos.lqr.R = 30.0*eye(2);
s2.pos.lqr.K = 350.0*eye(2);
s2.pos.lqr.K = -dlqr(s2.pos.Ad, s2.pos.Bd, s2.pos.lqr.Q, s2.pos.lqr.R);
s2.pos.lqi.I = 0.001*[0,-1;1,0];
s2.pos.lqi.max_integral = 10;
s2.pos.lqi.K = [s2.pos.lqr.K, s2.pos.lqi.I];


% Configuration 3
s3 = s;
s3.pos.lqr.Q = diag([1.0,1.0,0.3,0.3,0.001,0.001]);
s3.pos.lqr.R = 30.0*eye(2);
s3.pos.lqr.R = 15.0*eye(2);
s3.pos.lqr.K = -dlqr(s3.pos.Ad, s3.pos.Bd, s3.pos.lqr.Q, s3.pos.lqr.R);
s3.pos.lqi.I = 0.01 * [0, -1; 1, 0];
s3.pos.lqi.max_integral = 10;
s3.pos.lqi.K = [s3.pos.lqr.K, s3.pos.lqi.I];


% Configuration 4
s4 = s;
s4.pos.lqr.Q = diag([0.01,0.01,0.3,0.3,0.001,0.001]);
s4.pos.lqr.R = 30.0*eye(2);
s4.pos.lqr.R = 30.0*eye(2);
s4.pos.lqr.K = -dlqr(s4.pos.Ad, s4.pos.Bd, s4.pos.lqr.Q, s4.pos.lqr.R);
s4.pos.lqi.I = 0.01 * [0, -1; 1, 0];
s4.pos.lqi.max_integral = 10;
s4.pos.lqi.K = [s4.pos.lqr.K, s4.pos.lqi.I];

% Save the configurations
configs = [s1, s2, s3, s4];


%% Attitude

% Output to file
template = fileread(fullfile(inputfolder, 'AttitudeCodegenTemplate.cpp'));
outputFid = fopen('src-generated/AttitudeCodegen.cpp', 'w');
    
fprintf(outputFid, '/* Automatically generated, using... \r\n');
for k = 1:length(configs)
    
    % Current controller
    s = configs(k);
    
    % Attitude controller
    [controlSignal, integralWindup] = GenerateAttitudeController(s);
    controlSignalElements = symbolicVectorExpressionToStrings('Attitude', controlSignal); % Control output
    integralWindupElements = symbolicVectorExpressionToStrings('Attitude', integralWindup); % Integral increment

    % Attitude observer
    [prediction, innovation, stateEstimate] = GenerateAttitudeObserver(s);
    predictionElements = symbolicVectorExpressionToStrings('Attitude', prediction); % Prediction (reduced)
    innovationElements = symbolicVectorExpressionToStrings('Attitude', innovation); % Innovation (reduced)
    stateEstimateElements = symbolicVectorExpressionToStrings('Attitude', stateEstimate); % New stateEstimate (full)

    for i = 1:length(controlSignalElements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, controlSignalElements(i));
    end
    for i = 1:length(integralWindupElements)
        tag = strcat('$int', num2str(i - 1));
        template = replace(template, tag, integralWindupElements(i));
    end
    for i = 1:length(predictionElements)
        tag = strcat('$c', num2str(k), '$p', num2str(i - 0));
        template = replace(template, tag, predictionElements(i)); % reduced
    end
    for i = 1:length(innovationElements)
        tag = strcat('$c', num2str(k), '$i', num2str(i - 0));
        template = replace(template, tag, innovationElements(i)); % reduced
    end
    for i = 1:length(stateEstimateElements)
        tag = strcat('$x', num2str(i - 1));
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
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);


%% Altitude 

% Output to file
template = fileread(fullfile(inputfolder, 'AltitudeCodegenTemplate.cpp'));
outputFid = fopen('src-generated/AltitudeCodegen.cpp', 'w');

fprintf(outputFid, '/* Automatically generated, using \r\n');
for k = 1:length(configs)
    
    % Current configuration
    s = configs(k);
    
    % Altitude controller
    [controlSignal, integralWindup] = GenerateAltitudeController(s);
    controlSignalElements = symbolicVectorExpressionToStrings('Altitude', controlSignal); % Control output
    integralWindupElements = symbolicVectorExpressionToStrings('Altitude', integralWindup); % Integral increment

    % Altitude observer
    stateEstimate = GenerateAltitudeObserver(s);
    stateEstimateElements = symbolicVectorExpressionToStrings('Altitude', stateEstimate); % New stateEstimate

    for i = 1:length(controlSignalElements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, controlSignalElements(i));
    end
    for i = 1:length(integralWindupElements)
        tag = strcat('$int', num2str(i - 1));
        template = replace(template, tag, integralWindupElements(i));
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
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);


%% Position

%Output to file
template = fileread(fullfile(inputfolder, 'PositionCodegenTemplate.cpp'));
outputFid = fopen('src-generated/PositionCodegen.cpp', 'w');

fprintf(outputFid, '/* Automatically generated, using \r\n');
for k = 1:length(configs)
    
    % Current controller
    s = configs(k);
    
    % Position controller
    [controlSignal, integralWindup] = GeneratePositionController(s);
    controlSignalElements = symbolicVectorExpressionToStrings('Position', controlSignal); % Control output
    integralWindupElements = symbolicVectorExpressionToStrings('Position', integralWindup); % Integral increment

    % Position observer
    stateEstimate = GeneratePositionObserverBlind(s);
    stateEstimateElements = symbolicVectorExpressionToStrings('Position', stateEstimate);

    for i = 1:length(controlSignalElements)
        tag = strcat('$c', num2str(k), '$u', num2str(i - 1));
        template = replace(template, tag, controlSignalElements(i));
    end
    for i = 1:length(integralWindupElements)
        tag = strcat('$int', num2str(i - 1));
        template = replace(template, tag, integralWindupElements(i));
    end
    for i = 1:length(stateEstimateElements)
        tag = strcat('$x', num2str(i - 1));
        template = replace(template, tag, stateEstimateElements(i));
    end
    tag = strcat('$c', num2str(k), '$maxWindup');
    template = replace(template, tag, num2str(s.pos.lqi.max_integral));
    
    % Extend automatic signature with current controller
    fprintf(outputFid,  ' *\r\n');
    fprintf(outputFid, [' * Configuration ', char(num2str(k)), ': \r\n']);
    fprintf(outputFid,  ' * Q = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.pos.lqr.Q, 1)) '\r\n'], s.pos.lqr.Q');
    fprintf(outputFid,  ' * R = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.pos.lqr.R, 1)) '\r\n'], s.pos.lqr.R');
    fprintf(outputFid,  ' * I = \r\n');
    fprintf(outputFid, [' *  ' repmat(' %d', 1, size(s.pos.lqi.I, 1)) '\r\n'], s.pos.lqi.I');
    
end
fprintf(outputFid, ' *\r\n');
fprintf(outputFid, ' */\r\n\n');
fprintf(outputFid, template);
fclose(outputFid);


ExportParamsAndMatrices('ParamsAndMatrices-generated');

end



%% Symbolic expression to string
function textComponents = symbolicVectorExpressionToStrings(droneState, u)
u = vpa(u, 17);
u_str = char(u);
u_str = replace(u_str, 'matrix([[', '');
u_str = replace(u_str, ']])', '');

u_str = replace(u_str, '], [', ',');

regex = 'vector__(\w+?)(\d+)';
repl = 'vector__$1[$2 - 1]';
u_str = regexprep(u_str, regex, repl);
u_str = ReplaceWithStructParameterName(droneState, u_str);


regex = 'matrix__(\w+?)(\d+)_(\d+)';
repl = 'matrix__$1[$2 - 1][$3 - 1]';
u_str = regexprep(u_str, regex, repl);

textComponents = strsplit(u_str, ',');
end