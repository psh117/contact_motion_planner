#include <contact_motion_planner/solver/contact_optimization.h>

namespace suhan_contact_planner
{

Eigen::Matrix3d cross_skew(Eigen::Vector3d input)
{
  Eigen::Matrix3d output;
  output.setZero();
  output(0,1) = -input(2);
  output(1,0) = input(2);
  output(0,2) = input(1);
  output(2,0) = -input(1);
  output(1,2) = -input(0);
  output(2,1) = input(0);
  return output;
}

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

  const double contact_number = model_->getContactNumber();
  std::vector<ContactPtr> contacts;
  contacts = model_->getContactRobot();
  const auto& env_contact = model_->getContactEnvironment();
  contacts.insert(contacts.end(), env_contact.begin(), env_contact.end());

  if (contact_number == 0) return false;
  A.setZero(6, contact_number * 6);
  b.setZero(6);
  b.head<3>() =  model_->getTransform().linear() *
      Eigen::Vector3d(0,0,9.8) * model_->getMass();  // TODO: Check this
  // TODO: Momentum + b(3~5)
  for(size_t i=0; i<contact_number; i++)
  {
    A.block<3,3>(0, i*6).setIdentity();
    A.block<3,3>(3, i*6) = cross_skew(contacts[i]->getContactTransform().translation());
    A.block<3,3>(3, i*6+3).setIdentity();
  }
  eq_constraint->setA(A);
  eq_constraint->setEqualityCondition(b);

  solver.addConstraint(eq_constraint);


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
    R = contact->getContactTransform().linear().transpose();
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
      const auto& contact_length = contact->getContactLength();
      C_cop(0,2) =      contact_length[0] / 2;
      C_cop(1,2) = - (- contact_length[0] / 2);
      C_cop(2,2) =      contact_length[1] / 2;
      C_cop(3,2) = - (- contact_length[1] / 2);
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
  // ineq_constraint->printCondition();
  solver.addConstraint(ineq_constraint);
  solver.setContactNumber(model_->getContactNumber());
  Eigen::VectorXd result;
  if(solver.solve(result))
  {
    //auto &contacs = model_->getContactRobot();
    for(int i=0; i<contacts.size();i++)
    {
      // TODO: Force update!
      // TODO: Contact copy is needed!
      contacts[i]->setContactForceTorque(result.segment<6>(i*6));
    }
    return true;
  }
  return false;
}

} // namespace suhan_contact_planner
