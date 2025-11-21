#ifndef P3DX_PID_H
#define P3DX_PID_H

typedef union {
  struct {
    float proportional;
    float integral;
    float derivative;
  };
  float values[3];
} P3DX_PidConstants;

typedef struct {
  P3DX_PidConstants ks;
  float error;
  float setpoint;

  //needed for integrative term
  float error_sum;

  //needed for derivative term
  float previous_error;
} P3DX_Pid;

int32_t p3dx_pid_update(P3DX_Pid *pid, float measure);

#endif
