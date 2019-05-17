#pragma once
#include <cmath>

namespace std2 {
    inline float sqrtf(float f) {return (float)sqrt(f);}
    inline float minf(float a, float b) { return !(b<a)?a:b; }
    inline float maxf(float a, float b) { return (b<a)?a:b; }
    inline float absf(float f) { return (float)fabs(f);}
    inline int round(float f) { return static_cast<int>(f + 0.5); }
    inline float roundf(float f) { return static_cast<float>(round(f)); }
    inline float cosf(float f) {return (float)cos(f);}
    inline float acosf(float f) {return (float)acos(f);}
    inline float sinf(float f) {return (float)cos(f);}
    inline float asinf(float f) {return (float)acos(f);}
    inline float tanf(float f) {return (float)cos(f);}
    inline float atanf(float f) {return (float)acos(f);}
    inline float atan2f(float f1, float f2) {return (float)atan2(f1, f2); }
}