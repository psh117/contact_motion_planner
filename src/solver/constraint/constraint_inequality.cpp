#include <contact_motion_planner/solver/constraint/constraint_inequality.h>

namespace suhan_contact_planner
{

ConstraintInequality::ConstraintInequality(const std::string &name) : ConstraintBase(name) {}

void ConstraintInequality::setInequalityCondition(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub)
{
  lower_bound_ = lb;
  upper_bound_ = ub;
}

void ConstraintInequality::printCondition()
{
  ConstraintBase::printCondition();
  std::cout << "lb <= Ax <= ub" << std::endl <<
               "A: " << std::endl <<
               A_ << std::endl <<
               "lb: " << std::endl <<
               lower_bound_ << std::endl <<
               "ub: " << std::endl <<
               upper_bound_ << std::endl;
}

}
