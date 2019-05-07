#include <Time.hpp>

const real_t GTC_DURATION =
    (TICKS_PER_SECOND * 1);  //*** 1000ms from u_hover --> u_thrust_joystick

class GradualThrustChangeManager {
  private:
    bool busy;
    int counter;
    real_t thrust;

  public:
    bool isBusy();
    real_t getThrust();
    void init();
    void update();
};
