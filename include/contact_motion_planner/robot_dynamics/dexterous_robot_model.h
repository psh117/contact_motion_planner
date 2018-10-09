#pragma once

#include "contact_motion_planner/robot_dynamics/robot_dynamics_model.h"

namespace suhan_contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel
{

public:
  virtual bool isReachable(Eigen::Vector3d position) override
  {
    //ROS_INFO("%lf %lf %lf", position[0], position[1], position[2]);
    //std::cout << position.transpose() << std::endl;
    return (position.norm() < 0.5);
  }
  virtual bool isPossibleContact(Eigen::Isometry3d transform) override { return true; }

  // for grasp contact
  virtual Eigen::Matrix<double, 2, 6> getForceLimit() override
  {
    Eigen::Matrix<double, 2, 6> limit_matrix;
    //for(int )
    //limit_matrix
  }

};

}
