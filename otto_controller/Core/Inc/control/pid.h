#ifndef PID_H
#define PID_H

typedef union {
  struct {
    float proportional;
    float integral;
    float derivative;
  };
  struct {
    float kp;
    float ki;
    float kd;
  };
  float values[3];
} PidConstants;

typedef struct {
  PidConstants ks;
  float error;
  float setpoint;

  //needed for integrative term
  float error_sum;

  //needed for derivative term
  float previous_error;
} Pid;

int32_t pid_update(Pid *pid, float measure);

#endif
