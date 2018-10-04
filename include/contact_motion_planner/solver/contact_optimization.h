#pragma once

#include <limits>

#include <contact_motion_planner/contact_model/contact_model.h>
#include <contact_motion_planner/robot_dynamics/robot_dynamics_model.h>
#include <contact_motion_planner/solver/constraint/constraint_equality.h>
#include <contact_motion_planner/solver/constraint/constraint_inequality.h>
#include <contact_motion_planner/solver/contact_optimization_solver.h>

namespace suhan_contact_planner
{

class ContactOptimization
{
public:
  ContactOptimization();

  void setModel(ContactModelPtr model);
  void setRobot(RobotDynamicsModelPtr robot);
  void initializeConstraints();

private:
  ContactModelPtr model_;
  RobotDynamicsModelPtr robot_;
};

}
