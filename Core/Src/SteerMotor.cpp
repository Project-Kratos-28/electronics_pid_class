#include "SteerMotor.h"

SteerMotor::SteerMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                       GPIO_TypeDef* dirPort, uint16_t dirPin,
                       uint16_t homeRaw, int encoderDir, int motorDir,
                       float kp, float ki, float kd)
    // PID works in degrees. Steering never crosses +/-180 deg from home, so
    // the PID's own wrap handling isn't needed - folding happens in rawToAngle().
    : motor_(pwmTim, pwmChannel, pwmPeriod, dirPort, dirPin, 360.0f, kp, ki, kd),
      homeRaw_(homeRaw), encoderDir_(encoderDir >= 0 ? 1 : -1),
      target_(HOME_ANGLE), ramped_(HOME_ANGLE), maxRate_(DEFAULT_MAX_RATE),
      angle_(HOME_ANGLE), rampInit_(false)
{
    motor_.disableWrap();
    motor_.setDeadband(DEFAULT_DEADBAND);
    motor_.setMotorDirection(motorDir);
}

void SteerMotor::driveOpenLoop(float duty) {
    motor_.driveOpenLoop(duty);
    rampInit_ = false;
}

void SteerMotor::begin() {
    motor_.begin();
    rampInit_ = false;
}

void SteerMotor::stop() {
    motor_.stop();
    rampInit_ = false; // on restart, ramp from wherever the wheel actually is
}

void SteerMotor::setTargetAngle(float deg) {
    if (deg < MIN_ANGLE) deg = MIN_ANGLE;
    if (deg > MAX_ANGLE) deg = MAX_ANGLE;
    target_ = deg;
}

void SteerMotor::setGains(float kp, float ki, float kd) { motor_.setGains(kp, ki, kd); }
void SteerMotor::setDeadband(float deg)                 { motor_.setDeadband(deg); }
void SteerMotor::setMaxRate(float degPerSec)            { maxRate_ = (degPerSec > 0.0f) ? degPerSec : 0.0f; }
void SteerMotor::setHomeRaw(uint16_t homeRaw)           { homeRaw_ = homeRaw; rampInit_ = false; }

float SteerMotor::rawToAngle(uint16_t rawCounts) const {
    int rel = static_cast<int>(rawCounts & 0x0FFF) - static_cast<int>(homeRaw_);
    // Fold into [-2048, 2047] so the 4095 -> 0 rollover can sit anywhere.
    while (rel >= COUNTS_PER_REV / 2) rel -= COUNTS_PER_REV;
    while (rel < -COUNTS_PER_REV / 2) rel += COUNTS_PER_REV;
    return HOME_ANGLE + encoderDir_ * rel * (360.0f / COUNTS_PER_REV);
}

void SteerMotor::update(uint16_t rawCounts, float dt) {
    angle_ = rawToAngle(rawCounts);

    if (!rampInit_) {
        // Start the ramp at the wheel's real position so nothing jumps on boot/restart.
        ramped_ = angle_;
        if (ramped_ < MIN_ANGLE) ramped_ = MIN_ANGLE;
        if (ramped_ > MAX_ANGLE) ramped_ = MAX_ANGLE;
        rampInit_ = true;
    }

    // Slew-rate limit toward the controller's target.
    if (maxRate_ > 0.0f && dt > 0.0f) {
        float step = maxRate_ * dt;
        float diff = target_ - ramped_;
        if (diff > step)       ramped_ += step;
        else if (diff < -step) ramped_ -= step;
        else                   ramped_ = target_;
    } else {
        ramped_ = target_;
    }

    motor_.setTargetPosition(ramped_);
    motor_.update(angle_, dt);
}

float SteerMotor::getAngle() const        { return angle_; }
float SteerMotor::getTargetAngle() const  { return target_; }
float SteerMotor::getRampedTarget() const { return ramped_; }
float SteerMotor::getOutput() const       { return motor_.getOutput(); }
