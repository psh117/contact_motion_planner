#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <memory>

namespace suhan_contact_planner
{

class ConstraintBase
{
public:
  ConstraintBase(const std::string &name = "");

  const Eigen::MatrixXd & getA();
  const Eigen::VectorXd & getLowerBound();
  const Eigen::VectorXd & getUpperBound();

  void setA(const Eigen::MatrixXd & A);
  void setLowerBound(const Eigen::VectorXd & lb);
  void setUpperBound(const Eigen::VectorXd & ub);
  void setOnlyLowerBound(const Eigen::VectorXd & lb);
  void setOnlyUpperBound(const Eigen::VectorXd & ub);

  void resize(const size_t r, const size_t c);

  size_t rows();
  size_t cols();

  void checkDimension();

  virtual void printCondition();

  std::string name_;

protected:
  Eigen::MatrixXd A_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

};
typedef std::shared_ptr<ConstraintBase> ConstraintBasePtr;

} // namespace suhan_contact_planner
