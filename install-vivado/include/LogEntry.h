#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
#include <algorithm> // copy
#include <cstdint>   // uint32_t, int32_t
#include <iterator>  // begin, end

using float_6_const_ref = const float(&)[6];
using float_2_const_ref = const float(&)[2];
using float_4_const_ref = const float(&)[4];
using float_10_const_ref = const float(&)[10];
using float_3_const_ref = const float(&)[3];
#else
#include <stdint.h> // uint32_t, int32_t
#endif

/**
 * @brief   A struct for encapsulating all logging data. It has Python bindings,
 *          so it can be used with the GUI directly.
 * 
 * @note    This function is automatically generated by 
 *          `src/logentry/codegen/Codegen.py`.  
 *          Don't edit this file directly, edit the template instead: 
 *          `src/logentry/templates/LogEntry.template.h`.
 */
#ifdef __cplusplus
struct LogEntry {
    LogEntry() : size(getExpectedNumberOfWords()) {}

    uint32_t size;
    int32_t mode;
    uint64_t frametime;
    uint32_t framecounter;
    int32_t droneConfig;
    float rcTuning;
    float rcThrottle;
    float rcRoll;
    float rcPitch;
    float rcYaw;
    float referenceOrientation[4];
    float referenceOrientationEuler[3];
    float __pad0;
    float referenceHeight;
    float referenceLocation[2];
    float measurementOrientation[4];
    float measurementAngularVelocity[3];
    float measurementHeight;
    float measurementLocation[2];
    float attitudeObserverState[10];
    float altitudeObserverState[3];
    float navigationObserverState[6];
    float attitudeYawOffset;
    float attitudeControlSignals[3];
    float altitudeControlSignal;
    float positionControlSignal[2];
    float motorControlSignals[4];
    float commonThrust;
    float hoverThrust;

    constexpr static size_t getExpectedNumberOfWords() { return 64; }
    void setSize(uint32_t size) { this->size = size; }
    uint32_t getSize() const { return this->size; }
    void setMode(int32_t mode) { this->mode = mode; }
    int32_t getMode() const { return this->mode; }
    void setFrametime(uint64_t frametime) { this->frametime = frametime; }
    uint64_t getFrametime() const { return this->frametime; }
    void setFramecounter(uint32_t framecounter) { this->framecounter = framecounter; }
    uint32_t getFramecounter() const { return this->framecounter; }
    void setDroneConfig(int32_t droneConfig) { this->droneConfig = droneConfig; }
    int32_t getDroneConfig() const { return this->droneConfig; }
    void setRcTuning(float rcTuning) { this->rcTuning = rcTuning; }
    float getRcTuning() const { return this->rcTuning; }
    void setRcThrottle(float rcThrottle) { this->rcThrottle = rcThrottle; }
    float getRcThrottle() const { return this->rcThrottle; }
    void setRcRoll(float rcRoll) { this->rcRoll = rcRoll; }
    float getRcRoll() const { return this->rcRoll; }
    void setRcPitch(float rcPitch) { this->rcPitch = rcPitch; }
    float getRcPitch() const { return this->rcPitch; }
    void setRcYaw(float rcYaw) { this->rcYaw = rcYaw; }
    float getRcYaw() const { return this->rcYaw; }
    void setReferenceOrientation(float_4_const_ref referenceOrientation) {
        std::copy(std::begin(referenceOrientation),
              std::end(referenceOrientation), 
              std::begin(this->referenceOrientation));
    }
    float_4_const_ref getReferenceOrientation() const { return this->referenceOrientation; }
    void setReferenceOrientationEuler(float_3_const_ref referenceOrientationEuler) {
        std::copy(std::begin(referenceOrientationEuler),
              std::end(referenceOrientationEuler), 
              std::begin(this->referenceOrientationEuler));
    }
    float_3_const_ref getReferenceOrientationEuler() const { return this->referenceOrientationEuler; }
    void set__pad0(float __pad0) { this->__pad0 = __pad0; }
    float get__pad0() const { return this->__pad0; }
    void setReferenceHeight(float referenceHeight) { this->referenceHeight = referenceHeight; }
    float getReferenceHeight() const { return this->referenceHeight; }
    void setReferenceLocation(float_2_const_ref referenceLocation) {
        std::copy(std::begin(referenceLocation),
              std::end(referenceLocation), 
              std::begin(this->referenceLocation));
    }
    float_2_const_ref getReferenceLocation() const { return this->referenceLocation; }
    void setMeasurementOrientation(float_4_const_ref measurementOrientation) {
        std::copy(std::begin(measurementOrientation),
              std::end(measurementOrientation), 
              std::begin(this->measurementOrientation));
    }
    float_4_const_ref getMeasurementOrientation() const { return this->measurementOrientation; }
    void setMeasurementAngularVelocity(float_3_const_ref measurementAngularVelocity) {
        std::copy(std::begin(measurementAngularVelocity),
              std::end(measurementAngularVelocity), 
              std::begin(this->measurementAngularVelocity));
    }
    float_3_const_ref getMeasurementAngularVelocity() const { return this->measurementAngularVelocity; }
    void setMeasurementHeight(float measurementHeight) { this->measurementHeight = measurementHeight; }
    float getMeasurementHeight() const { return this->measurementHeight; }
    void setMeasurementLocation(float_2_const_ref measurementLocation) {
        std::copy(std::begin(measurementLocation),
              std::end(measurementLocation), 
              std::begin(this->measurementLocation));
    }
    float_2_const_ref getMeasurementLocation() const { return this->measurementLocation; }
    void setAttitudeObserverState(float_10_const_ref attitudeObserverState) {
        std::copy(std::begin(attitudeObserverState),
              std::end(attitudeObserverState), 
              std::begin(this->attitudeObserverState));
    }
    float_10_const_ref getAttitudeObserverState() const { return this->attitudeObserverState; }
    void setAltitudeObserverState(float_3_const_ref altitudeObserverState) {
        std::copy(std::begin(altitudeObserverState),
              std::end(altitudeObserverState), 
              std::begin(this->altitudeObserverState));
    }
    float_3_const_ref getAltitudeObserverState() const { return this->altitudeObserverState; }
    void setNavigationObserverState(float_6_const_ref navigationObserverState) {
        std::copy(std::begin(navigationObserverState),
              std::end(navigationObserverState), 
              std::begin(this->navigationObserverState));
    }
    float_6_const_ref getNavigationObserverState() const { return this->navigationObserverState; }
    void setAttitudeYawOffset(float attitudeYawOffset) { this->attitudeYawOffset = attitudeYawOffset; }
    float getAttitudeYawOffset() const { return this->attitudeYawOffset; }
    void setAttitudeControlSignals(float_3_const_ref attitudeControlSignals) {
        std::copy(std::begin(attitudeControlSignals),
              std::end(attitudeControlSignals), 
              std::begin(this->attitudeControlSignals));
    }
    float_3_const_ref getAttitudeControlSignals() const { return this->attitudeControlSignals; }
    void setAltitudeControlSignal(float altitudeControlSignal) { this->altitudeControlSignal = altitudeControlSignal; }
    float getAltitudeControlSignal() const { return this->altitudeControlSignal; }
    void setPositionControlSignal(float_2_const_ref positionControlSignal) {
        std::copy(std::begin(positionControlSignal),
              std::end(positionControlSignal), 
              std::begin(this->positionControlSignal));
    }
    float_2_const_ref getPositionControlSignal() const { return this->positionControlSignal; }
    void setMotorControlSignals(float_4_const_ref motorControlSignals) {
        std::copy(std::begin(motorControlSignals),
              std::end(motorControlSignals), 
              std::begin(this->motorControlSignals));
    }
    float_4_const_ref getMotorControlSignals() const { return this->motorControlSignals; }
    void setCommonThrust(float commonThrust) { this->commonThrust = commonThrust; }
    float getCommonThrust() const { return this->commonThrust; }
    void setHoverThrust(float hoverThrust) { this->hoverThrust = hoverThrust; }
    float getHoverThrust() const { return this->hoverThrust; }

};
#else
typedef struct {

    uint32_t size;
    int32_t mode;
    uint64_t frametime;
    uint32_t framecounter;
    int32_t droneConfig;
    float rcTuning;
    float rcThrottle;
    float rcRoll;
    float rcPitch;
    float rcYaw;
    float referenceOrientation[4];
    float referenceOrientationEuler[3];
    float __pad0;
    float referenceHeight;
    float referenceLocation[2];
    float measurementOrientation[4];
    float measurementAngularVelocity[3];
    float measurementHeight;
    float measurementLocation[2];
    float attitudeObserverState[10];
    float altitudeObserverState[3];
    float navigationObserverState[6];
    float attitudeYawOffset;
    float attitudeControlSignals[3];
    float altitudeControlSignal;
    float positionControlSignal[2];
    float motorControlSignals[4];
    float commonThrust;
    float hoverThrust;

} LogEntry;
#endif

#if !defined(__cplusplus) || defined(LOGGER_INCLUDE_C_WRAPPERS)
#ifdef __cplusplus
extern "C" {
#endif

uint32_t getSize(const LogEntry *logEntry);
void setSize(LogEntry *logEntry, uint32_t size);
int32_t getMode(const LogEntry *logEntry);
void setMode(LogEntry *logEntry, int32_t mode);
uint64_t getFrametime(const LogEntry *logEntry);
void setFrametime(LogEntry *logEntry, uint64_t frametime);
uint32_t getFramecounter(const LogEntry *logEntry);
void setFramecounter(LogEntry *logEntry, uint32_t framecounter);
int32_t getDroneConfig(const LogEntry *logEntry);
void setDroneConfig(LogEntry *logEntry, int32_t droneConfig);
float getRcTuning(const LogEntry *logEntry);
void setRcTuning(LogEntry *logEntry, float rcTuning);
float getRcThrottle(const LogEntry *logEntry);
void setRcThrottle(LogEntry *logEntry, float rcThrottle);
float getRcRoll(const LogEntry *logEntry);
void setRcRoll(LogEntry *logEntry, float rcRoll);
float getRcPitch(const LogEntry *logEntry);
void setRcPitch(LogEntry *logEntry, float rcPitch);
float getRcYaw(const LogEntry *logEntry);
void setRcYaw(LogEntry *logEntry, float rcYaw);
const float *getReferenceOrientation(const LogEntry *logEntry);
void setReferenceOrientation(LogEntry *logEntry, const float *referenceOrientation);
size_t getReferenceOrientationSize(void);
const float *getReferenceOrientationEuler(const LogEntry *logEntry);
void setReferenceOrientationEuler(LogEntry *logEntry, const float *referenceOrientationEuler);
size_t getReferenceOrientationEulerSize(void);
float get__pad0(const LogEntry *logEntry);
void set__pad0(LogEntry *logEntry, float __pad0);
float getReferenceHeight(const LogEntry *logEntry);
void setReferenceHeight(LogEntry *logEntry, float referenceHeight);
const float *getReferenceLocation(const LogEntry *logEntry);
void setReferenceLocation(LogEntry *logEntry, const float *referenceLocation);
size_t getReferenceLocationSize(void);
const float *getMeasurementOrientation(const LogEntry *logEntry);
void setMeasurementOrientation(LogEntry *logEntry, const float *measurementOrientation);
size_t getMeasurementOrientationSize(void);
const float *getMeasurementAngularVelocity(const LogEntry *logEntry);
void setMeasurementAngularVelocity(LogEntry *logEntry, const float *measurementAngularVelocity);
size_t getMeasurementAngularVelocitySize(void);
float getMeasurementHeight(const LogEntry *logEntry);
void setMeasurementHeight(LogEntry *logEntry, float measurementHeight);
const float *getMeasurementLocation(const LogEntry *logEntry);
void setMeasurementLocation(LogEntry *logEntry, const float *measurementLocation);
size_t getMeasurementLocationSize(void);
const float *getAttitudeObserverState(const LogEntry *logEntry);
void setAttitudeObserverState(LogEntry *logEntry, const float *attitudeObserverState);
size_t getAttitudeObserverStateSize(void);
const float *getAltitudeObserverState(const LogEntry *logEntry);
void setAltitudeObserverState(LogEntry *logEntry, const float *altitudeObserverState);
size_t getAltitudeObserverStateSize(void);
const float *getNavigationObserverState(const LogEntry *logEntry);
void setNavigationObserverState(LogEntry *logEntry, const float *navigationObserverState);
size_t getNavigationObserverStateSize(void);
float getAttitudeYawOffset(const LogEntry *logEntry);
void setAttitudeYawOffset(LogEntry *logEntry, float attitudeYawOffset);
const float *getAttitudeControlSignals(const LogEntry *logEntry);
void setAttitudeControlSignals(LogEntry *logEntry, const float *attitudeControlSignals);
size_t getAttitudeControlSignalsSize(void);
float getAltitudeControlSignal(const LogEntry *logEntry);
void setAltitudeControlSignal(LogEntry *logEntry, float altitudeControlSignal);
const float *getPositionControlSignal(const LogEntry *logEntry);
void setPositionControlSignal(LogEntry *logEntry, const float *positionControlSignal);
size_t getPositionControlSignalSize(void);
const float *getMotorControlSignals(const LogEntry *logEntry);
void setMotorControlSignals(LogEntry *logEntry, const float *motorControlSignals);
size_t getMotorControlSignalsSize(void);
float getCommonThrust(const LogEntry *logEntry);
void setCommonThrust(LogEntry *logEntry, float commonThrust);
float getHoverThrust(const LogEntry *logEntry);
void setHoverThrust(LogEntry *logEntry, float hoverThrust);

#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
static_assert(sizeof(LogEntry) == sizeof(float) * 64, 
              "Error: packing of LogEntry is incorrect");
#else
_Static_assert(sizeof(LogEntry) == sizeof(float) * 64, 
              "Error: packing of LogEntry is incorrect");
#endif

#endif  // LOGGER_H
