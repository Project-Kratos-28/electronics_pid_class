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

void MotorControl_Init(void);
void MotorControl_Update(float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
