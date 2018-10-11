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

bool ContactOptimizationSolver::solve(Eigen::VectorXd& result_force)
{
  size_t total_row = 0;
  for(auto & constraint : constraints)
  {
    total_row += constraint->rows();
    assert(contact_number_ * 6 == constraint->cols());
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
  int result;
  A_row_ = A_;
  if(!hot_start_)
  {
    result = qproblem_.init(H_row_.data(), g_.data(), A_row_.data(),
                   //lower_bound_.data(), upper_bound_.data(),
                            NULL,NULL,
                   A_lower_bound_.data(), A_upper_bound_.data(), iter);
    hot_start_ = true;
  }
  else
  {
    result = qproblem_.hotstart(H_row_.data(), g_.data(), A_row_.data(),
                       //lower_bound_.data(), upper_bound_.data(),
                                NULL,NULL,
                       A_lower_bound_.data(), A_upper_bound_.data(), iter);
  }
//  std::cout << "A: " << std:: endl
//            << A_ << std::endl;

//  std::cout << "A_lb.\': " << std:: endl
//            << A_lower_bound_.transpose() << std::endl;

//  std::cout << "A_ub.\': " << std:: endl
//            << A_upper_bound_.transpose() << std::endl;

  if (result == qpOASES::SUCCESSFUL_RETURN)
  {
    qproblem_.getPrimalSolution(x_solved_.data());
    result_force = x_solved_;
    //std::cout << x_solved_.transpose() << std::endl;
    qproblem_.reset();
    return true;
  }
  else
  {
    // std::cout << "qp solve failed: " << result << std::endl;
    return false;
  }

}

void ContactOptimizationSolver::resize(int total_row)
{
  A_.setZero(total_row, contact_number_ * 6);
  H_row_.setIdentity(contact_number_ * 6, contact_number_ * 6);
  A_lower_bound_.resize(total_row);
  A_upper_bound_.resize(total_row);
  g_.setZero(contact_number_ * 6);
  x_solved_.resize(contact_number_ * 6);
  qpOASES::Options options;

  options.setToDefault();
  options.initialStatusBounds = qpOASES::ST_INACTIVE;
  options.printLevel          = qpOASES::PL_NONE;
  options.enableRegularisation = qpOASES::BT_TRUE;
  options.enableEqualities = qpOASES::BT_TRUE;
  qproblem_ = qpOASES::SQProblem(contact_number_ * 6, total_row);
  qproblem_.setOptions(options);
}

} // namespace suhan_contact_planner
