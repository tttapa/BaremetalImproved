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
            string = 'integralWindup.q123.x';
        end
        if element == 2
            string = 'integralWindup.q123.y';
        end
        if element == 3
            string = 'integralWindup.q123.z';
        end
    end
    
    if strcmp(struct, 'x_hat')
        if element == 1
           string = 'stateEstimate.q.w';
        end
        if element == 2
           string = 'stateEstimate.q.x';
        end
        if element == 3
           string = 'stateEstimate.q.y';
        end
        if element == 4
           string = 'stateEstimate.q.z';
        end
        if element == 5
           string = 'stateEstimate.w.x';
        end
        if element == 6
           string = 'stateEstimate.w.y';
        end
        if element == 7
           string = 'stateEstimate.w.z';
        end
        if element == 8
           string = 'stateEstimate.n.x';
        end
        if element == 9
           string = 'stateEstimate.n.y';
        end
        if element == 10
           string = 'stateEstimate.n.z';
        end 
    end
    
    if strcmp(struct, 'ref')
       if element == 1
           string = 'reference.q.w';
       end
       if element == 2
           string = 'reference.q.x';
       end
       if element == 3
           string = 'reference.q.y';
       end
       if element == 4
           string = 'reference.q.z';
       end
    end
    
    if strcmp(struct, 'u') 
       if element == 1
           string = 'controlSignal.uxyz.x';
       end
       if element == 2
           string = 'controlSignal.uxyz.y';
       end
       if element == 3
           string = 'controlSignal.uxyz.z';
       end
    end
    
    if strcmp(struct, 'y')
       if element == 1 
          string = 'measurement.q.w';
       end
       if element == 2 
          string = 'measurement.q.x';
       end
       if element == 3 
          string = 'measurement.q.y';
       end
       if element == 4
          string = 'measurement.q.z';
       end
       if element == 5
          string = 'measurement.w.x';
       end
       if element == 6 
          string = 'measurement.w.y';
       end
       if element == 7
          string = 'measurement.w.z';
       end
    end
    
    if strcmp(struct, 'prediction')
       if element == 1
          string = 'prediction.q.w';
       end
       if element == 2
          string = 'prediction.q.x';
       end
       if element == 3
          string = 'prediction.q.y';
       end
       if element == 4
          string = 'prediction.q.z';
       end
       if element == 5
          string = 'prediction.w.x';
       end
       if element == 6
          string = 'prediction.w.y';
       end
       if element == 7
          string = 'prediction.w.z';
       end
       if element == 8
          string = 'prediction.n.x';
       end
       if element == 9
          string = 'prediction.n.y';
       end
       if element == 10
          string = 'prediction.n.z';
       end 
    end
    
    if strcmp(struct, 'innovation')
       if element == 1
          string = 'innovation.q.w';
       end
       if element == 2
          string = 'innovation.q.x';
       end
       if element == 3
          string = 'innovation.q.y';
       end
       if element == 4
          string = 'innovation.q.z';
       end
       if element == 5
          string = 'innovation.w.x';
       end
       if element == 6
          string = 'innovation.w.y';
       end
       if element == 7
          string = 'innovation.w.z';
       end
       if element == 8
          string = 'innovation.n.x';
       end
       if element == 9
          string = 'innovation.n.y';
       end
       if element == 10
          string = 'innovation.n.z';
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
            string = 'controlSignal.q12.x';
        end
        if element == 2
            string = 'controlSignal.q12.y';
        end
    end
     
end

end