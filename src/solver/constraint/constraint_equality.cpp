#include <contact_motion_planner/solver/constraint/constraint_equality.h>

namespace suhan_contact_planner
{

ConstraintEquality::ConstraintEquality(const std::string &name) : ConstraintBase(name) {}

void ConstraintEquality::setEqualityCondition(const Eigen::VectorXd & b)
{
  lower_bound_ = b;
  upper_bound_ = b;
}

void ConstraintEquality::printCondition()
{
  ConstraintBase::printCondition();
  std::cout << "Ax = b" << std::endl <<
               "A: " << std::endl <<
               A_ << std::endl <<
               "b: " << std::endl <<
               lower_bound_ << std::endl;
}

}
