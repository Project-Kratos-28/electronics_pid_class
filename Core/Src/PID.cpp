#include "PID.h"
float pTerm;
PID::PID(float kp, float ki, float kd, float outputMin, float outputMax)
    : kp_(kp), ki_(ki), kd_(kd),
      setpoint_(0.0f), integral_(0.0f), prevMeasurement_(0.0f),
      filteredD_(0.0f), // <--- INITIALIZE HERE
      outMin_(outputMin), outMax_(outputMax),
      firstRun_(true), wrapEnabled_(false), wrapRange_(0.0f)
{
}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float outMin, float outMax) {
    outMin_ = outMin;
    outMax_ = outMax;
}

void PID::setSetpoint(float setpoint) {
    setpoint_ = setpoint;
}

float PID::getSetpoint() const {
    return setpoint_;
}

void PID::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    filteredD_ = 0.0f; // <--- ADD THIS LINE
    firstRun_ = true;
}

void PID::enableWrap(float range) {
    wrapEnabled_ = true;
    wrapRange_ = range;
}

void PID::disableWrap() {
    wrapEnabled_ = false;
}

float PID::computeError(float measurement) const {
    float error = setpoint_ - measurement;

    if (wrapEnabled_ && wrapRange_ > 0.0f) {
        // Fold error into [-range/2, +range/2] so a 350 deg -> 10 deg move
        // is a 20 deg step, not a 340 deg one.
        float half = wrapRange_ * 0.5f;
        while (error > half)  error -= wrapRange_;
        while (error < -half) error += wrapRange_;
    }

    return error;
}

float PID::compute(float measurement, float dt) {
    if (dt <= 0.0f) {
        dt = 1e-3f; // guard against div-by-zero or bad timing on first call
    }

    float error = computeError(measurement);

    // --- Proportional ---
    pTerm = kp_ * error;

    // --- Integral ---
    integral_ += error * dt;
    float iTerm = ki_ * integral_;

    // --- Derivative on measurement with Low-Pass Filter ---
    // Avoids derivative "kick" on setpoint change and smooths encoder quantization noise.
    float dTerm = 0.0f;
    if (!firstRun_) {
        float dMeasurement = measurement - prevMeasurement_;
        if (wrapEnabled_ && wrapRange_ > 0.0f) {
            float half = wrapRange_ * 0.5f;
            while (dMeasurement > half)  dMeasurement -= wrapRange_;
            while (dMeasurement < -half) dMeasurement += wrapRange_;
        }
        
        // Raw derivative calculation
        float rawD = -(dMeasurement / dt);

        // First-Order Low-Pass Filter (alpha = 0.15 filters out 10ms encoder tick noise)
        float alpha = 0.15f;
        filteredD_ = (alpha * rawD) + ((1.0f - alpha) * filteredD_);

        dTerm = kd_ * filteredD_;
    }