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
            string = 'integralWindup.eul.yaw';
        end
        if element == 2
            string = 'integralWindup.eul.pitch';
        end
        if element == 3
            string = 'integralWindup.eul.roll';
        end
    end
    
    if strcmp(struct, 'x_hat')
        if element == 1
           string = 'stateEstimate.eul.yaw';
        end
        if element == 2
           string = 'stateEstimate.eul.pitch';
        end
        if element == 3
           string = 'stateEstimate.eul.roll';
        end
        if element == 4
           string = 'stateEstimate.wEul.yaw';
        end
        if element == 5
           string = 'stateEstimate.wEul.pitch';
        end
        if element == 6
           string = 'stateEstimate.wEul.roll';
        end
        if element == 7
           string = 'stateEstimate.nEul.yaw';
        end
        if element == 8
           string = 'stateEstimate.nEul.pitch';
        end
        if element == 9
           string = 'stateEstimate.nEul.roll';
        end
    end
    
    if strcmp(struct, 'ref')
       if element == 1
           string = 'reference.eul.yaw';
       end
       if element == 2
           string = 'reference.eul.pitch';
       end
       if element == 3
           string = 'reference.eul.roll';
       end
    end
    
    if strcmp(struct, 'u') 
       if element == 1
           string = 'controlSignal.uEul.yaw';
       end
       if element == 2
           string = 'controlSignal.uEul.pitch';
       end
       if element == 3
           string = 'controlSignal.uEul.roll';
       end
    end
    
    if strcmp(struct, 'y')
       if element == 1 
          string = 'measurement.eul.yaw';
       end
       if element == 2 
          string = 'measurement.eul.pitch';
       end
       if element == 3 
          string = 'measurement.eul.roll';
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
           string = 'stateEstimate.q.x';
       end
       if element == 2
           string = 'stateEstimate.q.y';
       end
       if element == 3
           string = 'stateEstimate.p.x';
       end
       if element == 4
           string = 'stateEstimate.p.y';
       end
       if element == 5
           string = 'stateEstimate.v.x';
       end
       if element == 6
           string = 'stateEstimate.v.y';
       end
    end
    
    if strcmp(struct, 'ref')
       if element == 1
          string = 'reference.p.x';
       end
       if element == 2
          string = 'reference.p.y';
       end
    end
    
    if strcmp(struct, 'y_int')
        if element == 1
            string = 'integralWindup.p.x';
        end
        if element == 2
            string = 'integralWindup.p.y';
        end
    end
    
    if strcmp(struct, 'u')
        if element == 1
            string = 'controlSignal.pitchRollRef.x';
        end
        if element == 2
            string = 'controlSignal.pitchRollRef.y';
        end
    end
     
end

end