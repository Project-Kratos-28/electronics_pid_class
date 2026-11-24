#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

/*
 * CubeMX regenerates main.c as plain C every time you touch the .ioc file,
 * so keep it C and just call into this thin extern "C" wrapper instead of
 * turning main.c itself into C++.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Elbow Position Motor (used by Python SWD Angle Tuner)
    extern volatile float g_kp;
    extern volatile float g_ki;
    extern volatile float g_kd;
    extern volatile float g_target_angle;
    extern volatile float g_current_angle;
    extern volatile float g_elbow_duty;

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

void Elbow_Control_Init(void);
void Elbow_Control_Update(float target_position, float dt_seconds);
void MotorControl_ProcessUARTChar(char c);
#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
