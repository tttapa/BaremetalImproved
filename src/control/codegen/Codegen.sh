#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
matlab -nodisplay -nosplash -nodesktop -nojvm -r \
"try, \
    addpath('$DIR'); \
    Codegen(); \
catch e, \
    disp(getReport(e)); \
    exit(1); \
end, \
exit(0);"