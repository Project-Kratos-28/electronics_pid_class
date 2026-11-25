#ifndef PID_H_
#define PID_H_

/*
 * Generic PID controller.
 * Used internally by both PositionMotor and SpeedMotor - not tied to any
 * specific hardware, so it's easy to unit-test on your PC before flashing.
 */
class PID {
public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float outMin, float outMax);
    void setSetpoint(float setpoint);
    float getSetpoint() const;

    void reset();

    // For circular/wrapping feedback, e.g. an absolute encoder that reports
    // 0-360 deg or 0-4095 counts and wraps around. Disabled by default -
    // only PositionMotor turns this on.
    void enableWrap(float range);
    void disableWrap();

    // Error band around the setpoint that counts as "on target", with
    // hysteresis: the controller settles once |error| <= deadband and stays
    // settled (output 0, integral cleared) until |error| > 2 x deadband.
    // Without the hysteresis, a reading that flickers by one encoder count at
    // the band edge flips the output sign every cycle and the motor buzzes.
    // Default 0 = off.
    void setDeadband(float deadband);

    // Time constant (seconds) of a first-order low-pass filter on the D term.
    // Encoder readings are quantised, so the raw derivative is spiky.
    // Default 0 = no filtering.
    void setDerivativeFilter(float tau);

    // Error from the most recent compute() call (after wrap folding).
    float getLastError() const;

    // measurement: current process value (position or speed)
    // dt: elapsed time in seconds since the previous call
    // returns: controller output, clamped to [outputMin, outputMax]
    float compute(float measurement, float dt);

private:
    float kp_;
    float ki_;
    float kd_;

    float setpoint_;
    float integral_;
    float prevMeasurement_;

    float outMin_;
    float outMax_;

    bool firstRun_;
    bool wrapEnabled_;
    float wrapRange_;

    float deadband_;
    bool settled_;
    float dFilterTau_;
    float dFiltered_;
    float lastError_;

    float computeError(float measurement) const;
};

#endif /* PID_H_ */
