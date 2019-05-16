function [ string ] = ReplaceWithStructParameterName(controllerName, string)
%REPLACE_WITH_STRUCT_PARAMETER_NAME
%   Replace the syms with the proper representation for the C++ code, namely
%   struct.member.
%
%   @param  controllerName
%           The name of the controller to that needs to replace its syms with
%           the proper struct.member representation. This should be either
%           'Attitude', 'Altitude' or 'Position'.
%   @param  string
%           The string containing the generated code with syms to replace.
%
%   @return string
%           The given string with all syms replaced by the proper struct.member.
%

vectors = strfind(string, 'vector__');

while (~isempty(vectors))

    % Parse the struct name.
    startIndex = vectors(1) + length('vector__');
    struct = char(extractBetween(string, startIndex, '['));

    % Parse the sym index and determine the struct member.
    startIndex = startIndex + length(struct) + 1;
    spaceIndices = strfind(string(startIndex+1:end),' ');
    endIndex = startIndex + spaceIndices(1) - 1;
    element = str2double(char(string(startIndex:endIndex)));
    parameterName = GetParameterName(controllerName, struct, element);

    % Replace the sym with the proper format struct.member.
    toReplace = char(extractBetween(string,vectors(1),']'));
    endIndex = vectors(1) + length(toReplace);
    toReplace = char(extractBetween(string, vectors(1), endIndex));  % TODO: necessary???
    string = strrep(string, toReplace, parameterName);

    % Find the rest of the vectors to replace
    vectors = strfind(string, 'vector__');
end

end

function string = GetParameterName(controllerName, struct, element)

if strcmp(controllerName, 'Attitude')
    
    if strcmp(struct,'y_int')
        if element == 1
            string = 'integralWindup.q123[0]';
        end
        if element == 2
            string = 'integralWindup.q123[1]';
        end
        if element == 3
            string = 'integralWindup.q123[2]';
        end
    end
    
    if strcmp(struct, 'x_hat')
        if element == 1
           string = 'stateEstimate.q[0]';
        end
        if element == 2
           string = 'stateEstimate.q[1]';
        end
        if element == 3
           string = 'stateEstimate.q[2]';
        end
        if element == 4
           string = 'stateEstimate.q[3]';
        end
        if element == 5
           string = 'stateEstimate.w[0]';
        end
        if element == 6
           string = 'stateEstimate.w[1]';
        end
        if element == 7
           string = 'stateEstimate.w[2]';
        end
        if element == 8
           string = 'stateEstimate.n[0]';
        end
        if element == 9
           string = 'stateEstimate.n[1]';
        end
        if element == 10
           string = 'stateEstimate.n[2]';
        end 
    end
    
    if strcmp(struct, 'ref')
       if element == 1
           string = 'reference.q[0]';
       end
       if element == 2
           string = 'reference.q[1]';
       end
       if element == 3
           string = 'reference.q[2]';
       end
       if element == 4
           string = 'reference.q[3]';
       end
    end
    
    if strcmp(struct, 'u') 
       if element == 1
           string = 'controlSignal.uxyz[0]';
       end
       if element == 2
           string = 'controlSignal.uxyz[1]';
       end
       if element == 3
           string = 'controlSignal.uxyz[2]';
       end
    end
    
    if strcmp(struct, 'y')
       if element == 1 
          string = 'measurement.q[0]';
       end
       if element == 2 
          string = 'measurement.q[1]';
       end
       if element == 3 
          string = 'measurement.q[2]';
       end
       if element == 4
          string = 'measurement.q[3]';
       end
       if element == 5
          string = 'measurement.w[0]';
       end
       if element == 6 
          string = 'measurement.w[1]';
       end
       if element == 7
          string = 'measurement.w[2]';
       end
    end
    
    if strcmp(struct, 'prediction')
       if element == 1
          string = 'prediction.q[0]';
       end
       if element == 2
          string = 'prediction.q[1]';
       end
       if element == 3
          string = 'prediction.q[2]';
       end
       if element == 4
          string = 'prediction.q[3]';
       end
       if element == 5
          string = 'prediction.w[0]';
       end
       if element == 6
          string = 'prediction.w[1]';
       end
       if element == 7
          string = 'prediction.w[2]';
       end
       if element == 8
          string = 'prediction.n[0]';
       end
       if element == 9
          string = 'prediction.n[1]';
       end
       if element == 10
          string = 'prediction.n[2]';
       end 
    end
    
    if strcmp(struct, 'innovation')
       if element == 1
          string = 'innovation.q[0]';
       end
       if element == 2
          string = 'innovation.q[1]';
       end
       if element == 3
          string = 'innovation.q[2]';
       end
       if element == 4
          string = 'innovation.q[3]';
       end
       if element == 5
          string = 'innovation.w[0]';
       end
       if element == 6
          string = 'innovation.w[1]';
       end
       if element == 7
          string = 'innovation.w[2]';
       end
       if element == 8
          string = 'innovation.n[0]';
       end
       if element == 9
          string = 'innovation.n[1]';
       end
       if element == 10
          string = 'innovation.n[2]';
       end 
    end  
end

if strcmp(controllerName, 'Altitude')
    
    if strcmp(struct, 'x_hat')
       if element == 1
          string = 'stateEstimate.nt'; 
       end
       if element == 2
          string = 'stateEstimate.z'; 
       end
       if element == 3
          string = 'stateEstimate.vz'; 
       end    
    end
    
    if strcmp(struct, 'ref')
        string = 'reference.z';
    end
    
    if strcmp(struct, 'y_int')
        string = 'integralWindup.z';
    end
    
    if strcmp(struct, 'u')
        string = 'controlSignal.ut';
    end
    
    if strcmp(struct, 'y')
        string = 'measurement.z';
    end
    
    if strcmp(struct, 'x_hat_copy')
        if element == 1
          string = 'stateEstimateCopy.nt'; 
       end
       if element == 2
          string = 'stateEstimateCopy.z'; 
       end
       if element == 3
          string = 'stateEstimateCopy.vz'; 
       end   
    end

end

if strcmp(controllerName, 'Position')
    
    if strcmp(struct, 'x_hat')
       if element == 1
           string = 'stateEstimate.q[0]';
       end
       if element == 2
           string = 'stateEstimate.q[1]';
       end
       if element == 3
           string = 'stateEstimate.p[0]';
       end
       if element == 4
           string = 'stateEstimate.p[1]';
       end
       if element == 5
           string = 'stateEstimate.v[0]';
       end
       if element == 6
           string = 'stateEstimate.v[1]';
       end
    end
    
    if strcmp(struct, 'ref')
       if element == 1
          string = 'reference.p[0]';
       end
       if element == 2
          string = 'reference.p[1]';
       end
    end
    
    if strcmp(struct, 'y_int')
        if element == 1
            string = 'integralWindup.p[0]';
        end
        if element == 2
            string = 'integralWindup.p[1]';
        end
    end
    
    if strcmp(struct, 'x_hat_blind_copy')
       if element == 1
           string = 'stateEstimateBlindCopy.p[0]';
       end
       if element == 2
           string = 'stateEstimateBlindCopy.p[1]';
       end
       if element == 3
           string = 'stateEstimateBlindCopy.v[0]';
       end
       if element == 4
           string = 'stateEstimateBlindCopy.v[1]';
       end
    end
    
    if strcmp(struct, 'u')
        if element == 1
            string = 'controlSignalBlind.q[0]';
        end
        if element == 2
            string = 'controlSignalBlind.q[1]';
        end
    end
     
end

end