#pragma once

#include <contact_motion_planner/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

class ConstraintEquality : public ConstraintBase
{
public:
  ConstraintEquality(const std::string &name = "");
  void setEqualityCondition(const Eigen::VectorXd & b);

  void printCondition() override;
};

} // namespace suhan_contact_planner

