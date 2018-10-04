#pragma once

#include <contact_motion_planner/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

class ConstraintInequality : public ConstraintBase
{

public:
  ConstraintInequality(const std::string &name = "");
  void setInequalityCondition(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub);

  virtual void printCondition() override;
};

} // namespace suhan_contact_planner
