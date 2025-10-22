#ifndef PID_H
#define PID_H

typedef struct {
  //PID constants
  float kp;
  float ki;
  float kd;

  float error;
  float setpoint;

  //needed for integrative term
  float error_sum;

  //needed for derivative term
  float previous_error;

  int32_t min;
  int32_t max;
} Pid;

int32_t pid_update(Pid *pid, float measure);

#endif
