#ifndef CONTACT_OPTIMIZATION_SOLVER_H
#define CONTACT_OPTIMIZATION_SOLVER_H

#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <memory>
#include <contact_motion_planner/contact_model/contact_model.h>
#include <contact_motion_planner/solver/constraint/constraint_base.h>

namespace suhan_contact_planner
{

typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> MatrixXdRow;


class ContactOptimizationSolver
{
public:
  ContactOptimizationSolver();
  void setContactNumber(int contact_number);
  void addConstraint(ConstraintBasePtr cb);
  bool solve();

private:
  qpOASES::SQProblem qproblem_;

  Eigen::MatrixXd H_, A_;
  Eigen::VectorXd g_;

  MatrixXdRow  H_row_, A_row_;
  Eigen::VectorXd A_lower_bound_, A_upper_bound_, lower_bound_, upper_bound_;

  size_t contact_number_;


  std::vector<ConstraintBasePtr> constraints;

  bool hot_start_{false};

  void resize(int total_row);
};

} // namespace suhan_contact_planner

#endif // CONTACT_OPTIMIZATION_SOLVER_H
