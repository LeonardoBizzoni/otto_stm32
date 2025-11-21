#ifndef P3DX_MOTOR_H
#define P3DX_MOTOR_H

typedef struct {
  GPIO_TypeDef *const sleep_gpio_port;
  GPIO_TypeDef *const dir_gpio_port;
  TIM_HandleTypeDef *const pwm_timer;
  uint32_t pwm_channel;
  int32_t max_dutycycle;
  uint16_t sleep_pin;
  uint16_t dir_pin;
} P3DX_Motor;

typedef uint8_t P3DX_MotorDirection;
enum {
  P3DX_MotorDirection_Backward = GPIO_PIN_RESET,
  P3DX_MotorDirection_Forward  = GPIO_PIN_SET,
};

void p3dx_motor_init(P3DX_Motor motors[P3DX_MOTOR_COUNT]);
void p3dx_motor_set_speed(P3DX_Motor *motor, int32_t duty_cycle);
void p3dx_motor_brake(P3DX_Motor *motor);
void p3dx_motor_coast(P3DX_Motor * motor);

#endif
