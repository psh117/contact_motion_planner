#include "contact_motion_planner/contact_model/contact_model.h"

#include <ros/ros.h>

namespace suhan_contact_planner
{

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}

bool ContactModel::isSamePose(const ContactModel& model, double threshold_x, double threshold_orientation) const
{
  Eigen::Vector3d orientation_diff;
  Eigen::Vector3d translation_diff;

  Eigen::Matrix3d rotation_diff = model.getTransform().linear() - transform_.linear();
  if (rotation_diff.squaredNorm() > 0.2) return false;

  translation_diff = model.getTransform().translation() - transform_.translation();
  orientation_diff =  getPhi(model.getTransform().linear(), transform_.linear());
  return (orientation_diff.norm() < threshold_orientation && translation_diff.norm() < threshold_x);
  //return (translation_diff.norm() < threshold);
}

void ContactModel::updateFCLModel()
{
  FCLEigenUtils::convertTransform(transform_, fcl_transform_);
}

void ContactModel::copyContactEnvironment()
{
  std::vector<ContactPtr>::iterator it;
  for (it = contact_environment_.begin(); it < contact_environment_.end(); it++)
  {
    ContactPtr new_contact = std::make_shared<Contact>(*(*it));
    *it = new_contact;
  }
}
void ContactModel::copyContactRobot()
{
  std::vector<ContactPtr>::iterator it;
  for (it = contact_robot_.begin(); it < contact_robot_.end(); it++)
  {
    ContactPtr new_contact = std::make_shared<Contact>(*(*it));
    *it = new_contact;
  }
}

void ContactModel::printContacts()
{
  std::cout << "## MAIN Transform: \n" <<
               transform_.matrix() << std:: endl;

  std::cout << "Contacts (Env): \n";
  for (auto g : contact_environment_)
  {
    g->printContactState();
  }
  std::cout << "Contacts (Robot): \n";
  for (auto g : contact_robot_)
  {
    g->printContactState();
  }
}

void ContactModel::printAbsoulteContactPositions()
{
  std::cout << "## MAIN Transform: \n" <<
               transform_.matrix() << std:: endl;

  std::cout << "Contacts (Robot, ABS): \n";
  for (auto g : contact_robot_)
  {
    std::cout << "[Contact State] \n" <<
                 "Transform: \n" <<
                 (transform_ * g->getContactTransform()).matrix() << std::endl <<
                 "Force: \n" <<
                 g->getContactForceTorque().transpose() << std::endl;
  }
}

}
