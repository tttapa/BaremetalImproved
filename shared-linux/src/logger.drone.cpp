#include <LogEntry.h>

#include <Configuration.hpp>
#include <Instances.hpp>
#include <RC.hpp>
#include <Time.hpp>

/**
 * @brief   Rather sketchy conversion from a struct of primitives of the same
 *          type to a reference of an array of this type.
 * 
 * @tparam  ArrayElementType
 *          The type of the elements of the result array.
 * @tparam  StructType
 *          The type of the struct to convert. Make sure that the struct is 
 *          well packed, and that the memory layout is consistent with that of
 *          an array. 
 */
template <class ArrayElementType = float, class StructType = void>
static ArrayElementType (&toCppArray(
    StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)] {
    static_assert(sizeof(StructType) % sizeof(ArrayElementType) == 0);
    return reinterpret_cast<
        ArrayElementType(&)[sizeof(StructType) / sizeof(ArrayElementType)]>(
        data);
}

/**
 * @brief   Rather sketchy conversion from a struct of primitives of the same
 *          type to a reference of an array of this type.
 * 
 * @tparam  ArrayElementType
 *          The type of the elements of the result array.
 * @tparam  StructType
 *          The type of the struct to convert. Make sure that the struct is 
 *          well packed, and that the memory layout is consistent with that of
 *          an array. 
 */
template <class ArrayElementType = float, class StructType = void>
static const ArrayElementType (&toCppArray(
    const StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)] {
    static_assert(sizeof(StructType) % sizeof(ArrayElementType) == 0);
    return reinterpret_cast<
        ArrayElementType(&)[sizeof(StructType) / sizeof(ArrayElementType)]>(data);
}

/**
 * @brief   Get the logging data of the entire Baremetal part. This function
 *          gets the logging data from all controller instances, as well as the
 *          RC inputs.
 * 
 * @note    This function is automatically generated by 
 *          `src/logentry/codegen/Codegen.py`.  
 *          Don't edit this file directly, edit the template instead: 
 *          `src/logentry/templates/logger.drone.template.cpp`.
 * 
 * @return  The logging data.
 */
LogEntry getLogData() {
    RCValues rc = readRC();

    LogEntry logEntry;
    logEntry.setSize(64);
    logEntry.setMode(getFlightMode());
    logEntry.setFrametime(getMillis());
    logEntry.setFramecounter(getTickCount());
    logEntry.setDroneConfig(configManager.getControllerConfiguration());
    logEntry.setRcTuning(getTuner());
    logEntry.setRcThrottle(getThrottle());
    logEntry.setRcRoll(getRoll());
    logEntry.setRcPitch(getPitch());
    logEntry.setRcYaw(getYaw());
    logEntry.setReferenceOrientation(toCppArray(attitudeController.getReferenceQuat()));
    logEntry.setReferenceOrientationEuler(toCppArray(attitudeController.getReferenceEuler()));
    logEntry.set__pad0({0});
    logEntry.setReferenceHeight(altitudeController.getReferenceHeight());
    logEntry.setReferenceLocation(toCppArray(position.getReferencePosition()));
    logEntry.setMeasurementOrientation(getAHRSQuat());
    logEntry.setMeasurementAngularVelocity(getGyroMeasurement());
    logEntry.setMeasurementHeight(getCorrectedSonarMeasurement());
    logEntry.setMeasurementLocation(getCorrectedPosition());
    logEntry.setAttitudeObserverState(toCppArray(attitude.getState()));
    logEntry.setAltitudeObserverState(toCppArray(altitude.getState()));
    logEntry.setNavigationObserverState(toCppArray(position.getState()));
    logEntry.setAttitudeYawOffset(getYawJump());
    logEntry.setAttitudeControlSignals(toCppArray(attitude.getControl()));
    logEntry.setAltitudeControlSignal(altitude.getControl());
    logEntry.setPositionControlSignal(position.getControl());
    logEntry.setMotorControlSignals({} /* TODO */);
    logEntry.setCommonThrust({} /* TODO */);
    logEntry.setHoverThrust(inputBias.getThrustBias());
 
    return logEntry;
}