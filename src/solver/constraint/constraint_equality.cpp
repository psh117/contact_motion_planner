#include <contact_motion_planner/solver/constraint/constraint_equality.h>

namespace suhan_contact_planner
{

ConstraintEquality::ConstraintEquality(const std::string &name) : ConstraintBase(name) {}

void ConstraintEquality::setEqualityCondition(const Eigen::VectorXd & b)
{
  //Eigen::VectorXd ones(b);
  //ones.setConstant(1);

  lower_bound_ = b;// - 10. * ones;
  upper_bound_ = b;// + 10. * ones;
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
