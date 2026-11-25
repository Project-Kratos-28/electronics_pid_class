#include "PID.h"
PID::PID(float kp, float ki, float kd, float outputMin, float outputMax)
    : kp_(kp), ki_(ki), kd_(kd),
      setpoint_(0.0f), integral_(0.0f), prevMeasurement_(0.0f),
      outMin_(outputMin), outMax_(outputMax),
      firstRun_(true), wrapEnabled_(false), wrapRange_(0.0f),
      deadband_(0.0f), settled_(false), dFilterTau_(0.0f), dFiltered_(0.0f), lastError_(0.0f)
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
    // A new target always gets acted on, even if it's within the hysteresis
    // band - otherwise small steering corrections (< 2x deadband) are ignored.
    if (setpoint != setpoint_) settled_ = false;
    setpoint_ = setpoint;
}

float PID::getSetpoint() const {
    return setpoint_;
}

void PID::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    dFiltered_ = 0.0f;
    lastError_ = 0.0f;
    settled_ = false;
    firstRun_ = true;
}

void PID::enableWrap(float range) {
    wrapEnabled_ = true;
    wrapRange_ = range;
}

void PID::disableWrap() {
    wrapEnabled_ = false;
}

void PID::setDeadband(float deadband) {
    deadband_ = (deadband > 0.0f) ? deadband : 0.0f;
}

void PID::setDerivativeFilter(float tau) {
    dFilterTau_ = (tau > 0.0f) ? tau : 0.0f;
}

float PID::getLastError() const {
    return lastError_;
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
    lastError_ = error;

    // --- Derivative on measurement, not on error ---
    // Avoids a derivative "kick" every time you change the setpoint.
    // Always updated (even inside the deadband) so the filter state and
    // prevMeasurement_ stay current and there's no spike on leaving it.
    float dTerm = 0.0f;
    if (!firstRun_) {
        float dMeasurement = measurement - prevMeasurement_;
        if (wrapEnabled_ && wrapRange_ > 0.0f) {
            float half = wrapRange_ * 0.5f;
            while (dMeasurement > half)  dMeasurement -= wrapRange_;
            while (dMeasurement < -half) dMeasurement += wrapRange_;
        }
        float dRaw = dMeasurement / dt;

        // First-order low-pass: alpha = dt / (tau + dt). tau = 0 -> alpha = 1 (no filter).
        float alpha = dt / (dFilterTau_ + dt);
        dFiltered_ += alpha * (dRaw - dFiltered_);

        dTerm = -kd_ * dFiltered_;
    }

    prevMeasurement_ = measurement;
    firstRun_ = false;

    // --- Deadband with hysteresis ---
    // Close enough to the target: stop driving and clear the integral so a
    // leftover value can't kick the motor back out. Only resume once the
    // error clearly leaves the band (2x), so one-count encoder flicker at
    // the edge doesn't toggle the motor on and off.
    if (deadband_ > 0.0f) {
        float absErr = (error >= 0.0f) ? error : -error;
        if (settled_) {
            if (absErr <= 2.0f * deadband_) return 0.0f;
            settled_ = false;
        } else if (absErr <= deadband_) {
            settled_ = true;
            integral_ = 0.0f;
            return 0.0f;
        }
    }

    // --- Proportional ---
    float pTerm = kp_ * error;

    // --- Integral ---
    integral_ += error * dt;
    float iTerm = ki_ * integral_;

    float output = pTerm + iTerm + dTerm;

    // --- Clamp + anti-windup ---
    // If we saturate, undo the integral step we just took so it doesn't
    // keep climbing while the motor is already maxed out.
    if (output > outMax_) {
        output = outMax_;
        if (ki_ != 0.0f) integral_ -= error * dt;
    } else if (output < outMin_) {
        output = outMin_;
        if (ki_ != 0.0f) integral_ -= error * dt;
    }

    return output;
}
