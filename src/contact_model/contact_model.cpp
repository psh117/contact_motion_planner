#include "contact_motion_planner/contact_model/contact_model.h"



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


bool ContactModel::operate(OperationDirection dir, double delta)
{
  if(contact_environment_.empty())  // No contact
  {
    if(dir == DIR_ROLL || dir == DIR_PITCH) return false;
  }
  // We assume that there's only a contact with environment
  else if(contact_environment_[0]->getContactState() == Contact::CONTACT_FACE)
  {
    // Face contact can do anything
  }
  else if(contact_environment_[0]->getContactState() == Contact::CONTACT_LINE)
  {
    if(dir == DIR_X || dir == DIR_Y || dir == DIR_YAW) return false;
  }
  else if(contact_environment_[0]->getContactState() == Contact::CONTACT_POINT)
  {
    if(dir == DIR_X || dir == DIR_Y) return false;
  }

  switch (dir)
  {
  case DIR_X:
    transform_.translation()(0) += delta;
    break;
  case DIR_Y:
    transform_.translation()(1) += delta;
    break;
  case DIR_Z:
    transform_.translation()(2) += delta;
    break;
  case DIR_ROLL:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitX());
    break;
  case DIR_PITCH:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitY());
    break;
  case DIR_YAW:
    transform_.linear() = transform_.linear() * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ());
    break;
  default:
    // Error
    return false;
  }

  updateFCLModel();
  return true;
}

bool ContactModel::isSamePose(const ContactModel& model, double threshold) const
{
  Eigen::Vector3d orientation_diff;
  Eigen::Vector3d translation_diff;

  orientation_diff =  getPhi(model.getTransform().linear(), transform_.linear());
  translation_diff = (model.getTransform().translation() - transform_.translation());

  return (orientation_diff.norm() < threshold && translation_diff.norm() < threshold);
}

void ContactModel::updateFCLModel()
{

}

}
