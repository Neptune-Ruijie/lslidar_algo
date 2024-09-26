#ifndef EULER_ANGLE_HPP
#define EULER_ANGLE_HPP

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace ship_gauge{


struct EulerAngle
{
  float roll;
  float pitch;
  float yaw;

//   EulerAngle() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
  EulerAngle(float p = 0.0f, float r = 0.0f, float y = 0.0f) : roll(r), pitch(p), yaw(y) {}
  
};

}

#endif  // POINT_CLOUD_REGISTRATION_HPP