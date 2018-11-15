#pragma once

#include "contact_motion_planner/robot_dynamics/robot_dynamics_model.h"

namespace suhan_contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel
{
public:
  bool isReachable(Eigen::Vector3d position) override
  {
    //ROS_INFO("%lf %lf %lf", position[0], position[1], position[2]);
    //std::cout << position.transpose() << std::endl;
    return (position.norm() < 1.5);
  }
  bool isPossibleContact(Eigen::Affine3d transform) override { return true; }

  Eigen::Vector3d getMaximumForce();
  Eigen::Vector3d getMaximumMoment();
  // for grasp contact
  Eigen::Matrix<double, 2, 6> getForceLimit() override
  {
    Eigen::Matrix<double, 2, 6> limit_matrix;
    //for(int )
    //limit_matrix
  }
};

}
