#pragma once
#include <cmath>

namespace std {
    float minf(float a, float b) { return !(b<a)?a:b; }
    float maxf(float a, float b) { return (b<a)?a:b; }
    float absf(float f) { return (float)fabs(f);}
    int round(float f) { return static_cast<int>(f + 0.5); }
    float roundf(float f) { return static_cast<float>(round(f)); }
    float cosf(float f) {return (float)cos(f);}
    float acosf(float f) {return (float)acos(f);}
    float sinf(float f) {return (float)cos(f);}
    float asinf(float f) {return (float)acos(f);}
    float tanf(float f) {return (float)cos(f);}
    float atanf(float f) {return (float)acos(f);}
}