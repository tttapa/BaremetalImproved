// Original: BareMetal/src/control/attitude.h
#include <Matrix.hpp>
#include <Quaternion.hpp>
#include <EulerAngles.hpp>
#include <attitude-controller.hpp>

// ! Siel: Monday night we decided to use structs instead of wrapped ColVectors (the classes below)
// !       so this should be reverted to the lines commented out, but I don't think we're going to use
// !       Quat32, so this should probably be replaced by q0;q1;q2;q3
// struct AttitudeReference {
//     Quat32 q;   // Orientation
// };

// struct AttitudeState {
//     Quat32 q;   // Orientation
//     float wx;   // X angular velocity (rad/s)
//     float wy;   // Y angular velocity (rad/s)
//     float wz;   // Z angular velocity (rad/s)
//     float nx;   // X motor angular velocity (rad/s)
//     float ny;   // Y motor angular velocity (rad/s)
//     float nz;   // Z motor angular velocity (rad/s)
// };

// struct AttitudeControlSignal {
//     float ux;   // X motor signal (/)
//     float uy;   // Y motor signal (/)
//     float uz;   // Z motor signal (/)
// };

class DroneAttitudeState {
    ColVector<10> x = {1};

  public:
    DroneAttitudeState() = default;
    DroneAttitudeState(const ColVector<10> &x) : x{x} {}

    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(x); }
    EulerAngles getOrientationEuler() const {
        return getOrientation();
    }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(x); }
    ColVector<3> getMotorSpeed() const { return getBlock<7, 10, 0, 1>(x); }

    void setOrientation(const Quaternion &q) { assignBlock<0, 4, 0, 1>(x) = q; }
    void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(x) = w;
    }
    void setMotorSpeed(const ColVector<3> &n) {
        assignBlock<7, 10, 0, 1>(x) = n;
    }
    operator ColVector<10>() const { return x; }
};

auto &toCppArray(DroneAttitudeState x) {
    return toCppArray(ColVector<10>(x));
}

class Altitude {
  public:
    Altitude(int configuratie) : config(configuratie) {}

    ColVector<3> getControllerOutput(Quaternion reference) {
        ColVector<3> u_att;
        getAttitudeControllerOutput(
            toCppArray(x), toCppArray(reference.asColVector()),
            toCppArray(u_att), toCppArray(integral), config, 0);
        return u_att;
    }

    void reset() {
        x = {};
        integral = {};
        // TODO: reset observer
    }

  private:
    DroneAttitudeState x;
    ColVector<3> integral;
    int config;
};