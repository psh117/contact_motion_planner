#include "contact_motion_planner/solver/contact_optimization_solver.h"


namespace suhan_contact_planner
{

ContactOptimizationSolver::ContactOptimizationSolver()
{}

void ContactOptimizationSolver::addConstraint(ConstraintBasePtr cb)
{
  constraints.push_back(cb);
}

void ContactOptimizationSolver::setContactNumber(int contact_number)
{ contact_number_ = contact_number; }

bool ContactOptimizationSolver::solve()
{
  size_t total_row = 0;
  for(auto & constraint : constraints)
  {
    total_row += constraint->rows();
    assert(contact_number_ * 2 == constraint->cols());
  }

  resize(total_row);

  size_t A_row_index = 0;
  for(auto & constraint : constraints)
  {
    A_.block(A_row_index, 0, constraint->rows(), constraint->cols()) =
        constraint->getA();
    A_lower_bound_.segment(A_row_index, constraint->rows()) =
        constraint->getLowerBound();
    A_upper_bound_.segment(A_row_index, constraint->rows()) =
        constraint->getUpperBound();
    A_row_index += constraint->rows();
  }

  int iter = 1000;
  A_row_ = A_;
  if(!hot_start_)
  {
    qproblem_.init(H_row_.data(), g_.data(), A_row_.data(),
                   lower_bound_.data(), upper_bound_.data(),
                   A_lower_bound_.data(), A_upper_bound_.data(), iter);
    hot_start_ = true;
  }
  else
  {
    qproblem_.hotstart(H_row_.data(), g_.data(), A_row_.data(),
                       lower_bound_.data(), upper_bound_.data(),
                       A_lower_bound_.data(), A_upper_bound_.data(), iter);
  }

}

void ContactOptimizationSolver::resize(int total_row)
{
  A_.setZero(total_row, contact_number_ * 2);
  H_row_.setIdentity(contact_number_ * 2, contact_number_ * 2);
  g_.setZero(contact_number_ * 2);
}

} // namespace suhan_contact_planner
