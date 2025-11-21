#ifndef P3DX_ENCODER_H
#define P3DX_ENCODER_H

typedef struct {
  TIM_HandleTypeDef *const timer;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks;
  int32_t ticks_per_revolution;
  float wheel_circumference;
} P3DX_Encoder;

void  p3dx_encoder_init(P3DX_Encoder encoders[P3DX_ENCODER_COUNT]);
void  p3dx_encoder_update(P3DX_Encoder *encoder);
float p3dx_encoder_get_linear_velocity(const P3DX_Encoder *encoder);

void    p3dx_encoder_count_reset(P3DX_Encoder *encoder);
int32_t p3dx_encoder_count_get(const P3DX_Encoder *encoder);

#endif
