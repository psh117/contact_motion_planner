#include "contact_motion_planner/solver/constraint/constraint_base.h"


namespace suhan_contact_planner
{

ConstraintBase::ConstraintBase(const std::string &name)
  : name_(name)
{}

const Eigen::MatrixXd & ConstraintBase::getA()
{ return A_; }
const Eigen::VectorXd & ConstraintBase::getLowerBound()
{ return lower_bound_; }
const Eigen::VectorXd & ConstraintBase::getUpperBound()
{ return upper_bound_; }

void ConstraintBase::setA(const Eigen::MatrixXd &A)
{ A_ = A; }
void ConstraintBase::setLowerBound(const Eigen::VectorXd & lb)
{ lower_bound_ = lb; }
void ConstraintBase::setUpperBound(const Eigen::VectorXd & ub)
{ upper_bound_ = ub; }

void ConstraintBase::setOnlyLowerBound(const Eigen::VectorXd & lb)
{
  Eigen::VectorXd ub;
  ub.setConstant(lb.rows(), std::numeric_limits<double>::max());
  setLowerBound(lb);
  setUpperBound(ub);
}
void ConstraintBase::setOnlyUpperBound(const Eigen::VectorXd & ub)
{
  Eigen::VectorXd lb;
  lb.setConstant(ub.rows(), std::numeric_limits<double>::lowest());
  setLowerBound(lb);
  setUpperBound(ub);
}

void ConstraintBase::resize(const size_t r, const size_t c)
{
  A_.setIdentity(r,c);
  upper_bound_.setZero(r);
  lower_bound_.setZero(r);
}

size_t ConstraintBase::rows()
{
  checkDimension();
  return A_.rows();
}
size_t ConstraintBase::cols()
{
  checkDimension();
  return A_.cols();
}

void ConstraintBase::checkDimension()
{
  assert(A_.rows() == lower_bound_.rows() && lower_bound_.rows() == upper_bound_.rows());
}

void ConstraintBase::printCondition()
{
  std::cout << "[Constraint name]: " << name_ <<
               "-----------------------------------" <<
               std::endl;
}

} // namespace suhan_contact_planner
