#include <contact_motion_planner/solver/contact_optimization.h>

namespace suhan_contact_planner
{

void ContactOptimization::setModel(ContactModelPtr model)
{ model_ = model; }

void ContactOptimization::setRobot(RobotDynamicsModelPtr robot)
{ robot_ = robot; }

bool ContactOptimization::solve()
{
  ContactOptimizationSolver solver;

  auto eq_constraint = std::make_shared<ConstraintEquality>();
  // eq_constraint.
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  // TODO: gravity
  const auto model_R = model_->getTransform().linear();

  const double contact_number = model_->getContactNumber();
  Eigen::Vector3d gravity;
  gravity << 0, 0, 9.8;
  A.setZero(6, contact_number * 6);
  b.setZero(6);
  b.head<3>() = model_R * gravity * model_->getMass();  // TODO: Check this
  // TODO: Momentum + b(3~5)
  for(size_t i=0; i<contact_number; i++)
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

  // every contact
  auto ineq_constraint = std::make_shared<ConstraintInequality>();

  Eigen::MatrixXd C_all;
  Eigen::VectorXd d_all;

  C_all.setZero(11*contact_number, 6*contact_number);
  d_all.setZero(contact_number * 11);

  int i=0;
  for(auto & contact : contacts)
  {
    Eigen::Matrix<double, 6, 6> R_hat;
    Eigen::Matrix<double, 3, 3> R;
    R = contact->getContactTransform().linear();
    R_hat.setZero();
    R_hat.block<3,3>(0,0) = R;
    R_hat.block<3,3>(3,3) = R;

    // graspless
    Eigen::MatrixXd C_nrm;
    Eigen::VectorXd d_nrm;
    C_nrm.setZero(1,6);
    C_nrm(2) = 1;
    d_nrm.setZero(1);

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


    Eigen::MatrixXd C_i;
    C_i.setZero(11,6);
    C_i.block<1,6>(0, 0) = C_nrm;
    C_i.block<4,6>(1, 0) = C_cop;
    C_i.block<6,6>(1+4, 0) = C_frc;


    C_all.block<11,6>(i*11,i*6) = C_i * R_hat;
    d_all.segment<1>(i*11) = d_nrm;
    d_all.segment<4>(i*11+1) = d_cop;
    d_all.segment<6>(i*11+1+4) = d_frc;
        //C_all.block<11,6>(i*11,i*6) =
    //solver.addConstraint(graspless_contact_constraint);
    //solver.addConstraint(friction_constraint);
    //solver.addConstraint(cop_constraint);

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
    i++;
  }
  ineq_constraint->setA(C_all);
  ineq_constraint->setOnlyLowerBound(d_all);
  solver.addConstraint(ineq_constraint);
  solver.setContactNumber(model_->getContactNumber());
  return solver.solve();
}

} // namespace suhan_contact_planner
