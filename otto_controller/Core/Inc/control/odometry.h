#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct {
  union {
    struct {
      float right;
      float left;
    } setpoint;
    float setpoints[2];
    struct {
      float right;
      float left;
    } velocity;
    float velocites[2];
  };

  float linear_velocity;
  float angular_velocity;
  float baseline;
} Odometry;

void odometry_setpoint_from_cmdvel(Odometry *odom, float linear_vel,
                                   float angular_vel) {
  odom->setpoint.left = linear_vel - (odom->baseline * angular_vel) / 2;
  odom->setpoint.right = linear_vel + (odom->baseline * angular_vel) / 2;
}

#if 0
class Odometry {
  // NOTE(lb): can i delete this?
  void FromWheelVelToOdom(float left_wheel_vel, float right_wheel_vel){
    linear_velocity_ = (left_wheel_vel + right_wheel_vel)/2;
    angular_velocity_ = (right_wheel_vel - left_wheel_vel)/baseline_;
  }
};
#endif

#endif
