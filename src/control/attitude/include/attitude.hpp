// Original: BareMetal/src/control/attitude.h
#include <Matrix.hpp>
#include <Quaternion.hpp>
#include <EulerAngles.hpp>
#include <attitude-controller.hpp>

struct AttitudeReference {
    Quaternion q = Quaternion(1,0,0,0);   // Orientation
};

struct AttitudeState {
    Quaternion q = Quaternion(1,0,0,0);  // Orientation
    real_t wx;     // X angular velocity (rad/s)
    real_t wy;     // Y angular velocity (rad/s)
    real_t wz;     // Z angular velocity (rad/s)
    real_t nx;     // X motor angular velocity (rad/s)
    real_t ny;     // Y motor angular velocity (rad/s)
    real_t nz;     // Z motor angular velocity (rad/s)
};

struct AttitudeControlSignal {
    real_t ux;   // X motor signal (/)
    real_t uy;   // Y motor signal (/)
    real_t uz;   // Z motor signal (/)
};

struct AttitudeIntegralAction {
    real_t q1;
    real_t q2;
    real_t q3;
};

struct AttitudeMeasurement {
    Quaternion q;
    real_t wx;     // X angular velocity (rad/s)
    real_t wy;     // Y angular velocity (rad/s)
    real_t wz;     // Z angular velocity (rad/s)
};

class Attitude {

  private:
    AttitudeControlSignal u;
    AttitudeIntegralAction y_int;
    AttitudeReference ref;
    AttitudeMeasurement y;
    real_t uz_clamp;
    real_t thrust_clamp;
    void clampAttitudeControllerOutput(AttitudeControlSignal,real_t);

  public: 
    AttitudeState x_hat;
    void updateController();
    void initializeController();
    void idleController();
};
