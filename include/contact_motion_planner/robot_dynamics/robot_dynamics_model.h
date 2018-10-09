#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace suhan_contact_planner
{

class RobotDynamicsModel
{
public:
  virtual bool isReachable(Eigen::Vector3d position)=0;
  virtual bool isPossibleContact(Eigen::Affine3d transform)=0;
  virtual Eigen::Matrix<double, 2, 6> getForceLimit()=0;
};

typedef std::shared_ptr<RobotDynamicsModel> RobotDynamicsModelPtr;

}
