#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t previous_millis;
  uint32_t current_millis;
  int32_t ticks;  //if negative the wheel is going backwards
  int32_t ticks_per_revolution;
  float wheel_circumference;
} Encoder;

void encoder_init(Encoder *encoders);
void encoder_update(Encoder *encoder);
float encoder_linear_velocity(Encoder *encoder);

void encoder_count_reset(Encoder *encoder);
int encoder_count_get(Encoder *encoder);

inline float meters_from_ticks(float encoder_ticks,
                               float wheel_circumference,
                               float ticks_per_revolution);

#endif
