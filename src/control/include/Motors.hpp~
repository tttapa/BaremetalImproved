// Original: BareMetal/src/control/attitude.h

#include <real_t.h>
#include <attitude.hpp>

struct MotorSignal {
    real_t v0;
    real_t v1;
    real_t v2;
    real_t v3;
};

class Motor {
  private:
    MotorSignal v;
    real_t thrust_clamp;
    void calculateMotorSignal(Attitude attitude);
}

