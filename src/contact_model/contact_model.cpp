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


bool ContactModel::operate(OperationDirection dir, double delta_x, double delta_orientation)
{
  if(contact_environment_.empty())  // No contact
  {
    if(dir == DIR_ROLL || dir == DIR_PITCH) return false;
  }
  // We assume that there's only a contact with environment
  else if(contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_FACE)
  {
    // Face contact can do anything
  }
  else if(contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_LINE)
  {
    if(dir == DIR_X || dir == DIR_Y || dir == DIR_YAW) return false;
  }
  else if(contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_POINT)
  {
    if(dir == DIR_X || dir == DIR_Y) return false;
  }

  // TODO: When a contact model object is operated, contact state should be changed
  //       You may change the class structure
  switch (dir)
  {
  case DIR_X:
    transform_.translation()(0) += delta_x;
    break;
  case DIR_Y:
    transform_.translation()(1) += delta_x;
    break;
  case DIR_Z:
    transform_.translation()(2) += delta_x;
    break;

  // TODO: The rotation should be rotated w.r.t contact point not the center of the object.
  case DIR_ROLL:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitX());
    break;
  case DIR_PITCH:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitY());
    break;
  case DIR_YAW:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitZ());
    break;
  default:
    // Error
    return false;
  }

  updateFCLModel();
  return true;
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

}
