#ifndef P3DX_ODOMETRY_H
#define P3DX_ODOMETRY_H

typedef struct {
  float baseline;
  float velocity_linear;
  float velocity_angular;
  float setpoint_right;
  float setpoint_left;
} P3DX_Odometry;

#endif
