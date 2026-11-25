#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdint.h>

#define NUM_ENCODERS 4   // AS5600s on PCA9548A channels 0..3

/*
 * CubeMX regenerates main.c as plain C every time you touch the .ioc file,
 * so keep it C and just call into this thin extern "C" wrapper instead of
 * turning main.c itself into C++.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Steering motor (used by Python SWD Angle Tuner). Angles: 0..180 deg, 90 = straight.
    extern volatile float g_kp;
    extern volatile float g_ki;
    extern volatile float g_kd;
    extern volatile float g_target_angle;
    extern volatile float g_current_angle;
    extern volatile float g_steer_duty;
    extern volatile uint32_t g_steer_raw;
    extern volatile uint32_t g_steer_fault;
    extern volatile float g_steer_test_duty;
    extern volatile uint32_t g_steer_test_ms;

    // All four AS5600 encoders (PCA9548A channels 0..3)
    extern volatile uint32_t g_enc_raw[NUM_ENCODERS];
    extern volatile uint32_t g_enc_err[NUM_ENCODERS];
    extern volatile uint32_t g_enc_ok;
    extern volatile uint32_t g_control_dt_ms;

    // Drive Speed Motor (independent SWD variables)
    extern volatile float g_drive_kp;
    extern volatile float g_drive_ki;
    extern volatile float g_drive_kd;
    extern volatile float g_target_rpmf;
    extern volatile float g_current_rpm;
    extern volatile float g_drive_duty;


void MotorControl_Init(void);
void MotorControl_Update(float dt_seconds);
void MotorControl_SetParams(float kp, float ki, float kd, float targetRPM);

// Steering: no timer interrupt - call Steer_Control_Task() from the main loop.
void Steer_Control_Init(void);
void Steer_Control_Task(void);
// deg: 0..180 from the Ackermann controller (90 = straight ahead)
void Steer_SetTargetAngle(float deg);
// Sweep all 4 encoders into g_enc_raw[] (also done inside Steer_Control_Task)
void Encoders_ReadAll(void);
void MotorControl_ProcessUARTChar(char c);
#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
