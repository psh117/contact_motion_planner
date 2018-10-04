#include <contact_motion_planner/solver/contact_optimization.h>

namespace suhan_contact_planner
{

void ContactOptimization::setModel(ContactModelPtr model)
{ model_ = model; }

void ContactOptimization::setRobot(RobotDynamicsModelPtr robot)
{ robot_ = robot; }

void ContactOptimization::initializeConstraints()
{
  ContactOptimizationSolver solver;

  auto eq_constraint = std::make_shared<ConstraintEquality>();
  // eq_constraint.
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  // TODO: gravity
  A.setZero(6, model_->getContactNumber() * 2);
  b.setZero(6);
  b(2) = -9.8;
  // TODO: Momentum + b(3~5)
  for(size_t i=0; i<model_->getContactNumber(); i++)
  {
    A.block<3,3>(0, i*6).setIdentity();
    A.block<3,3>(3, i*6+3).setIdentity();
  }
  eq_constraint->setA(A);
  eq_constraint->setEqualityCondition(b);
  solver.addConstraint(eq_constraint);

  std::vector<ContactPtr> contacts;
  contacts = model_->getContactRobot();
  const auto& env_contact = model_->getContactEnvironment();
  contacts.insert(contacts.end(), env_contact.begin(), env_contact.end());

  for(auto & contact : contacts)
  {
    Eigen::Matrix<double, 6, 6> R_hat;
    Eigen::Matrix<double, 3, 3> R;
    R = contact->getContactTransform().linear();
    R_hat.setZero();
    R_hat.block<3,3>(0,0) = R;
    R_hat.block<3,3>(3,3) = R;

    // every contact
    auto graspless_contact_constraint
         = std::make_shared<ConstraintInequality>();
    auto friction_constraint
         = std::make_shared<ConstraintInequality>();
    auto cop_constraint
         = std::make_shared<ConstraintInequality>();

    // graspless
    Eigen::MatrixXd C_nrm;
    Eigen::VectorXd d_nrm;
    C_nrm.setZero(1,6);
    C_nrm(2) = 1;
    d_nrm.setZero(1);
    graspless_contact_constraint->setA(C_nrm * R_hat);
    graspless_contact_constraint->setOnlyLowerBound(d_nrm);

    // friction
    Eigen::MatrixXd C_frc;
    Eigen::VectorXd d_frc;
    double mu = model_->getFriction();

    C_frc.setZero(6, 6);
    for(int i=0; i<3; i++)
    {
      C_frc(i*2,   i) = -1;
      C_frc(i*2+1, i) =  1;
      C_frc(i*2,   2) = mu;
      C_frc(i*2+1, 2) = mu;
    }
    C_frc(4,5) = -1;
    C_frc(5,5) =  1;
    d_frc.setZero(6);
    friction_constraint->setA(C_frc * R_hat);
    friction_constraint->setOnlyLowerBound(d_frc);

    // CoP
    Eigen::MatrixXd C_cop;
    Eigen::VectorXd d_cop;
    C_cop.setZero(4, 6);
    d_cop.setZero(4);
    C_cop(0, 4) = 1;
    C_cop(1, 4) = -1;
    C_cop(2, 3) = -1;
    C_cop(3, 3) = 1;
    if (contact->getContactState() == Contact::ContactState::CONTACT_FACE)
    {
      // l_x l_y
    }
    cop_constraint->setA(C_cop * R_hat);
    cop_constraint->setOnlyLowerBound(d_cop);

    solver.addConstraint(graspless_contact_constraint);
    solver.addConstraint(friction_constraint);
    solver.addConstraint(cop_constraint);

    // Grasp limit
    /*
    Eigen::MatrixXd C_max[3];
    Eigen::VectorXd d_max[3];
    const auto & robot_->getForceLimit();
    for(int i=0; i<3; i++)
    {
      C_max.setZero(2, 6);
      C_max[i](0,i) = 1;
      C_max[i](1,i) = -1;

      d_max.resize(2);

    }
    */
  }

  solver.solve();

}

} // namespace suhan_contact_planner
