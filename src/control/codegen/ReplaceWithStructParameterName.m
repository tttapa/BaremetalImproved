function string = ReplaceWithStructParameterName(droneState, string)

vectors = strfind(string, 'vector__');
%matrices = strfind(string, 'matrix__');

while (~isempty(vectors))
    start = vectors(1) + 8;
    struct = char(extractBetween(string, start, '['));
    startIndex = start + length(struct) + 1;
    spaceIndices = strfind(string(startIndex+1:end),' ');
    endIndex = startIndex + spaceIndices(1) - 1;
    element = str2double(char(string(startIndex:endIndex)));
    parameterName = GetParameterName(droneState, struct, element);

    toReplace = char(extractBetween(string,vectors(1),']'));
    endIndex = vectors(1) + length(toReplace);
    toReplace = char(extractBetween(string, vectors(1), endIndex));

    string = strrep(string, toReplace, parameterName);
    vectors = strfind(string, 'vector__');
end
end

function string = GetParameterName(droneState, struct, element)

if strcmp(droneState, 'Attitude')

    if strcmp(struct,'y_int')
        if element == 1
            string = 'integralWindup.q1';
        end
        if element == 2
            string = 'integralWindup.q2';
        end
        if element == 3
            string = 'integralWindup.q3';
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
           string = 'stateEstimate.wx';
        end
        if element == 6
           string = 'stateEstimate.wy';
        end
        if element == 7
           string = 'stateEstimate.wz';
        end
        if element == 8
           string = 'stateEstimate.nx';
        end
        if element == 9
           string = 'stateEstimate.ny';
        end
        if element == 10
           string = 'stateEstimate.nz';
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
           string = 'controlSignal.ux';
       end
       if element == 2
           string = 'controlSignal.uy';
       end
       if element == 3
           string = 'controlSignal.uz';
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
          string = 'measurement.wx';
       end
       if element == 6 
          string = 'measurement.wy';
       end
       if element == 7
          string = 'measurement.wz';
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
          string = 'prediction.wx';
       end
       if element == 6
          string = 'prediction.wy';
       end
       if element == 7
          string = 'prediction.wz';
       end
       if element == 8
          string = 'prediction.nx';
       end
       if element == 9
          string = 'prediction.ny';
       end
       if element == 10
          string = 'prediction.nz';
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
          string = 'innovation.wx';
       end
       if element == 6
          string = 'innovation.wy';
       end
       if element == 7
          string = 'innovation.wz';
       end
       if element == 8
          string = 'innovation.nx';
       end
       if element == 9
          string = 'innovation.ny';
       end
       if element == 10
          string = 'innovation.nz';
       end 
    end  
end

if strcmp(droneState, 'Altitude')
    
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

if strcmp(droneState, 'Position')
    
    if strcmp(struct, 'x_hat')
       if element == 1
           string = 'stateEstimate.q1';
       end
       if element == 2
           string = 'stateEstimate.q2';
       end
       if element == 3
           string = 'stateEstimate.p.x';
       end
       if element == 4
           string = 'stateEstimate.p.y';
       end
       if element == 5
           string = 'stateEstimate.vx';
       end
       if element == 6
           string = 'stateEstimate.vy';
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
            string = 'integralWindup.x';
        end
        if element == 2
            string = 'integralWindup.y';
        end
    end
    
    if strcmp(struct, 'x_hat_blind_copy')
       if element == 1
           string = 'stateEstimateBlindCopy.p.x';
       end
       if element == 2
           string = 'stateEstimateBlindCopy.p.y';
       end
       if element == 3
           string = 'stateEstimateBlindCopy.vx';
       end
       if element == 4
           string = 'stateEstimateBlindCopy.vy';
       end
    end
    
    if strcmp(struct, 'u')
        if element == 1
            string = 'controlSignalBlind.q1';
        end
        if element == 2
            string = 'controlSignalBlind.q2';
        end
    end
     
end

end