#ifndef DEXTEROUS_ROBOT_MODEL_H
#define DEXTEROUS_ROBOT_MODEL_H

#include "contact_motion_planner/robot_dynamics/robot_dynamics_model.h"

namespace suhan_contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel
{

public:
  virtual bool isReachable(Eigen::Vector3d position)
  {
    //ROS_INFO("%lf %lf %lf", position[0], position[1], position[2]);
    //std::cout << position.transpose() << std::endl;
    return (position.norm() < 1.0);
  }
  virtual bool isPossibleContact(Eigen::Isometry3d transform) { return true; }
};

}
#endif // DEXTEROUS_ROBOT_MODEL_H
