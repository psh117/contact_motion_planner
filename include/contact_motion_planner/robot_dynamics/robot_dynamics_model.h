#ifndef ROBOT_DYNAMICS_MODEL_H
#define ROBOT_DYNAMICS_MODEL_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

namespace suhan_contact_planner
{


class RobotDynamicsModel
{
public:
  virtual bool isReachable(Eigen::Vector3d position)=0;
  virtual bool isPossibleContact(Eigen::Isometry3d transform)=0;

};

typedef std::shared_ptr<RobotDynamicsModel> RobotDynamicsModelPtr;

}
#endif
