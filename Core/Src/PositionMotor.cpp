#include "PositionMotor.h"
<<<<<<< HEAD

PositionMotor::PositionMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                              GPIO_TypeDef* dirPort, uint16_t dirPin1, uint16_t dirPin2,
                              float positionRange,
                              float kp, float ki, float kd)
    : pwmTim_(pwmTim), pwmChannel_(pwmChannel), pwmPeriod_(pwmPeriod),
      dirPort_(dirPort), dirPin1_(dirPin1), dirPin2_(dirPin2),
      pid_(kp, ki, kd, -100.0f, 100.0f),
      lastOutput_(0.0f)
{
    pid_.enableWrap(positionRange); // shortest-path error for an absolute encoder
=======
#include <cmath>

PositionMotor::PositionMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                              GPIO_TypeDef* dirPort, uint16_t dirPin,
                              float positionRange,
                              float kp, float ki, float kd)
    : pwmTim_(pwmTim), pwmChannel_(pwmChannel), pwmPeriod_(pwmPeriod),
      dirPort_(dirPort), dirPin_(dirPin),
      pid_(kp, ki, kd, -100.0f, 100.0f),
      lastOutput_(0.0f)
{
    pid_.enableWrap(positionRange); // Enabled by default for rotary motors
}

void PositionMotor::disableWrap() {
    pid_.disableWrap(); // Disables angle-wrap calculations
>>>>>>> d4f14b2 (Updated project code for Nucleo-F410RB live PID tuner)
}

void PositionMotor::begin() {
    HAL_TIM_PWM_Start(pwmTim_, pwmChannel_);
<<<<<<< HEAD
=======
    __HAL_TIM_MOE_ENABLE(pwmTim_); // Explicitly enable main PWM output for TIM1
>>>>>>> d4f14b2 (Updated project code for Nucleo-F410RB live PID tuner)
    stop();
}

void PositionMotor::setTargetPosition(float positionSetpoint) {
    pid_.setSetpoint(positionSetpoint);
}

void PositionMotor::setGains(float kp, float ki, float kd) {
<<<<<<< HEAD
    pid_.setGains(kp, ki, kd);                                                                                                                                                                                                                  































    
=======
    pid_.setGains(kp, ki, kd);
>>>>>>> d4f14b2 (Updated project code for Nucleo-F410RB live PID tuner)
}

void PositionMotor::update(float currentPosition, float dt) {
    float output = pid_.compute(currentPosition, dt);
    lastOutput_ = output;
    driveOutput(output);
}

void PositionMotor::stop() {
<<<<<<< HEAD
    HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dirPort_, dirPin2_, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, 0);
    lastOutput_ = 0.0f;
}

float PositionMotor::getOutput() const {
    return lastOutput_;
}

float PositionMotor::getTargetPosition() const {
    return pid_.getSetpoint();
}

void PositionMotor::driveOutput(float pidOutput) {
    // Direction
    if (pidOutput >= 0.0f) {
        HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dirPort_, dirPin2_, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(dirPort_, dirPin1_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dirPort_, dirPin2_, GPIO_PIN_SET);
    }

    // Magnitude -> duty cycle
    float duty = (pidOutput < 0.0f) ? -pidOutput : pidOutput; // 0..100
    if (duty > 100.0f) duty = 100.0f;

=======
	HAL_GPIO_WritePin(dirPort_, dirPin_, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, 0);
	lastOutput_ = 0.0f;
}

float PositionMotor::getOutput() const
{
	return lastOutput_;
}
float PositionMotor::getTargetPosition() const
{
	return pid_.getSetpoint();
}

void PositionMotor::driveOutput(float pidOutput) {
    // 1. Direction logic
    if (pidOutput >= 0.0f) {
        HAL_GPIO_WritePin(dirPort_, dirPin_, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dirPort_, dirPin_, GPIO_PIN_RESET);
    }

    // 2. Magnitude & Stiction threshold
    float duty = std::abs(pidOutput);
    if (duty > 100.0f) duty = 100.0f;

    const float MIN_DUTY = 25.0f; // Overcomes static motor friction
    if (duty > 0.1f && duty < MIN_DUTY) {
        duty = MIN_DUTY;
    }

>>>>>>> d4f14b2 (Updated project code for Nucleo-F410RB live PID tuner)
    uint32_t compareValue = static_cast<uint32_t>((duty / 100.0f) * static_cast<float>(pwmPeriod_));
    __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, compareValue);
}
