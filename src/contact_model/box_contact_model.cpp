#include "contact_motion_planner/contact_model/box_contact_model.h"


namespace suhan_contact_planner
{


BoxContactModel::BoxContactModel(const std::string &name, const Eigen::Vector3d &dimension) :
  ContactModel(name), dimension_(dimension)
{
  fcl_model_ = std::make_shared<fcl::Box>(dimension_(0),
                                          dimension_(1),
                                          dimension_(2));
}

BoxContactModel::BoxContactModel(const Eigen::Vector3d &dimension) : dimension_(dimension)
{
  BoxContactModel("", dimension);
}

void BoxContactModel::createContactSamples(std::vector <ContactPtr> &contact_samples)
{
  // TODO: Check the orientation of the contact
  // Predefined matrix
  const Eigen::Matrix3d rot_x_90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d rot_y_90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  const Eigen::Matrix3d rot_z_90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

  const Eigen::Matrix3d rot_x_m90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d rot_y_m90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY());
  const Eigen::Matrix3d rot_z_m90 = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
  // Contact points
  // Up and down with vertices
  for(size_t i=0; i < point_samples_per_edge_; i++)
  {
    for(size_t j=0; j < point_samples_per_edge_; j++)
    {
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      ContactPtr new_point_up = std::make_shared<Contact>();
      ContactPtr new_point_down = std::make_shared<Contact>();
      transform.translation()(0) =  i * dimension_(0) / (point_samples_per_edge_ - 1) - dimension_(0) / 2;
      transform.translation()(1) =  j * dimension_(1) / (point_samples_per_edge_ - 1) - dimension_(1) / 2;
      transform.translation()(2) =  -dimension_(2) / 2;
      transform.linear() = Eigen::Matrix3d::Identity();
      new_point_down->setTransform(transform);
      transform.translation()(2) =  dimension_(2) / 2;
      transform.linear() = Eigen::Matrix3d::Identity() *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
      new_point_up->setTransform(transform);
      contact_samples.push_back(new_point_up);
      contact_samples.push_back(new_point_down);
    }
  }
  // Front and rear
  for(size_t i=1; i < point_samples_per_edge_-1; i++)
  {
    for(size_t j=0; j < point_samples_per_edge_; j++)
    {
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      ContactPtr new_point_front = std::make_shared<Contact>();
      ContactPtr new_point_rear = std::make_shared<Contact>();
      transform.translation()(0) =  j * dimension_(0) / (point_samples_per_edge_ - 1) - dimension_(0) / 2;
      transform.translation()(2) =  i * dimension_(2) / (point_samples_per_edge_ - 1) - dimension_(2) / 2;
      transform.translation()(1) =  dimension_(1) / 2;
      transform.linear() = rot_x_90;
      new_point_front->setTransform(transform);
      transform.translation()(1) =  - dimension_(1) / 2;
      transform.linear() = rot_x_m90;
      new_point_rear->setTransform(transform);
      contact_samples.push_back(new_point_front);
      contact_samples.push_back(new_point_rear);
    }
  }
  // Left and right
  for(size_t i=1; i < point_samples_per_edge_-1; i++)
  {
    for(size_t j=1; j < point_samples_per_edge_-1; j++)
    {
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      ContactPtr new_point_left = std::make_shared<Contact>();
      ContactPtr new_point_right = std::make_shared<Contact>();
      transform.translation()(1) =  i * dimension_(1) / (point_samples_per_edge_ - 1) - dimension_(1) / 2;
      transform.translation()(2) =  j * dimension_(2) / (point_samples_per_edge_ - 1) - dimension_(2) / 2;
      transform.translation()(0) =  dimension_(0) / 2;
      transform.linear() = rot_y_m90;
      new_point_left->setTransform(transform);
      transform.translation()(0) =  - dimension_(0) / 2;
      transform.linear() = rot_y_90;
      new_point_right->setTransform(transform);
      contact_samples.push_back(new_point_left);
      contact_samples.push_back(new_point_right);
    }
  }

  // Contact Line
  for(size_t i=0; i<line_samples_per_side_; i++)
  {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    ContactPtr new_line[12];
    for(int i=0; i<12; i++)
      new_line[i] = std::make_shared<Contact>(Contact::ContactState::CONTACT_LINE);

    // Up and down
    ContactPtr new_line_down = std::make_shared<Contact>(Contact::ContactState::CONTACT_LINE);
    transform.linear() = Eigen::Matrix3d::Identity();
    transform.translation()(0) = dimension_(0) / (line_samples_per_side_+1) * (i+1) - dimension_(0)/2;
    transform.translation()(1) = 0;
    transform.translation()(2) = dimension_(2) / 2;
    new_line[0]->setTransform(transform);
    transform.translation()(2) = -dimension_(2) / 2;
    new_line[1]->setTransform(transform);

    transform.linear() = rot_z_90;
    transform.translation()(0) = 0;
    transform.translation()(1) = dimension_(1) / (line_samples_per_side_+1) * (i+1) - dimension_(1)/2;
    transform.translation()(2) = dimension_(2) / 2;
    new_line[2]->setTransform(transform);
    transform.translation()(2) = -dimension_(2) / 2;
    new_line[3]->setTransform(transform);

    // Front and rear
    transform.linear() = rot_x_90;
    transform.translation()(0) = 0;
    transform.translation()(2) = dimension_(2) / (line_samples_per_side_+1) * (i+1) - dimension_(2)/2;

    transform.translation()(1) = dimension_(1) / 2;
    new_line[4]->setTransform(transform);
    transform.translation()(1) = -dimension_(1) / 2;
    new_line[5]->setTransform(transform);

    transform.linear() = rot_x_90 * rot_z_90;
    transform.translation()(2) = 0;
    transform.translation()(0) = dimension_(0) / (line_samples_per_side_+1) * (i+1) - dimension_(0)/2;

    transform.translation()(1) = dimension_(1) / 2;
    new_line[6]->setTransform(transform);
    transform.translation()(1) = -dimension_(1) / 2;
    new_line[7]->setTransform(transform);

    // Left and right
    transform.linear() = rot_y_90;
    transform.translation()(2) = 0;
    transform.translation()(1) = dimension_(1) / (line_samples_per_side_+1) * (i+1) - dimension_(1)/2;

    transform.translation()(0) = dimension_(0) / 2;
    new_line[8]->setTransform(transform);
    transform.translation()(0) = -dimension_(0) / 2;
    new_line[9]->setTransform(transform);

    transform.linear() = rot_y_90 * rot_z_90;
    transform.translation()(1) = 0;
    transform.translation()(2) = dimension_(2) / (line_samples_per_side_+1) * (i+1) - dimension_(2)/2;

    transform.translation()(0) = dimension_(0) / 2;
    new_line[10]->setTransform(transform);
    transform.translation()(0) = -dimension_(0) / 2;
    new_line[11]->setTransform(transform);

    for(int i=0; i<12; i++)
      contact_samples.push_back(new_line[i]);
  }

}

bool BoxContactModel::operate(OperationDirection dir, double delta_x, double delta_orientation)
{
  Eigen::Affine3d transform_operated = Eigen::Affine3d::Identity();
  Eigen::Affine3d transform_obj_c = Eigen::Affine3d::Identity();
  Eigen::Affine3d transform_c_c2 = Eigen::Affine3d::Identity();
  Eigen::Affine3d transform_0_c2 = Eigen::Affine3d::Identity();
  Eigen::Affine3d transform_0_c2_p = Eigen::Affine3d::Identity();
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
    transform_operated.translation()(0) = delta_x;
    transform_ = transform_ * transform_operated;
    break;
  case DIR_Y:
    transform_operated.translation()(1) = delta_x;
    transform_ = transform_ * transform_operated;
    break;
  case DIR_Z: // Should I consider putting it down again?
    transform_operated.translation()(2) = delta_x;
    transform_ = transform_ * transform_operated;
    contact_environment_.clear();
    break;

    // TODO: The rotation should be rotated w.r.t contact point not the center of the object.
  case DIR_ROLL:  // along X axis

    if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_POINT)
    {
      transform_obj_c = contact_environment_[0]->getContactTransform();
      transform_0_c2 = transform_ * transform_obj_c;
      transform_0_c2_p = transform_0_c2;
      transform_0_c2_p.linear() = transform_0_c2_p.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitX());
      transform_ = transform_0_c2_p * transform_obj_c.inverse();
    }
    else
    {
      if(delta_orientation > 0.0)
      {
        transform_c_c2.translation()(1) = - dimension_(1) / 2;
      }
      else
      {
        transform_c_c2.translation()(1) = dimension_(1) / 2;
      }
      transform_obj_c = contact_environment_[0]->getContactTransform();
      transform_0_c2 = transform_ * transform_obj_c * transform_c_c2;
      transform_0_c2_p = transform_0_c2;
      transform_0_c2_p.linear() = transform_0_c2_p.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitX());
      transform_ = transform_0_c2_p * transform_c_c2.inverse() * transform_obj_c.inverse();
      contact_environment_[0]->setTransform(transform_obj_c * transform_c_c2);
    }
    if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_LINE &&
        line_contact_direction_ != dir) // Pivooooot
    {
      //std::cout << "pivot" << std::endl;
      contact_environment_[0]->setContactState(Contact::ContactState::CONTACT_POINT);
    }
    else if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_FACE ||
             (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_LINE &&
              line_contact_direction_ == dir))  // Rotate
    {
      contact_environment_[0]->setContactState(Contact::ContactState::CONTACT_LINE);
      line_contact_direction_ = DIR_ROLL;
    }

    break;
  case DIR_PITCH: // along Y axis
    if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_POINT)
    {
      transform_obj_c = contact_environment_[0]->getContactTransform();
      transform_0_c2 = transform_ * transform_obj_c;
      transform_0_c2_p = transform_0_c2;
      transform_0_c2_p.linear() = transform_0_c2_p.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitY());
      transform_ = transform_0_c2_p * transform_obj_c.inverse();
    }
    else
    {
      if(delta_orientation > 0.0)
      {
        transform_c_c2.translation()(0) = dimension_(0) / 2;
      }
      else
      {
        transform_c_c2.translation()(0) = - dimension_(0) / 2;
      }
      transform_obj_c = contact_environment_[0]->getContactTransform();
      transform_0_c2 = transform_ * transform_obj_c * transform_c_c2;
      transform_0_c2_p = transform_0_c2;
      transform_0_c2_p.linear() = transform_0_c2_p.linear() * Eigen::AngleAxisd(delta_orientation, Eigen::Vector3d::UnitY());
      transform_ = transform_0_c2_p * transform_c_c2.inverse() * transform_obj_c.inverse();
      contact_environment_[0]->setTransform(transform_obj_c * transform_c_c2);

    }
    if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_LINE &&
        line_contact_direction_ != dir) // Pivooooot
    {
      //std::cout << "pivot" << std::endl;
      contact_environment_[0]->setContactState(Contact::ContactState::CONTACT_POINT);
    }
    else if (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_FACE ||
             (contact_environment_[0]->getContactState() == Contact::ContactState::CONTACT_LINE &&
              line_contact_direction_ == dir))  // Rotate
    {
      contact_environment_[0]->setContactState(Contact::ContactState::CONTACT_LINE);
      line_contact_direction_ = DIR_PITCH;
    }
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

ContactPtr BoxContactModel::getBottomContact()
{
  ContactPtr contact = std::make_shared<Contact>(Contact::ContactState::CONTACT_FACE, Contact::ContactRelation::CONTACT_OBJ_ENV);
  Eigen::Affine3d transform;

  transform.translation()(0) =  0;
  transform.translation()(1) =  0;
  transform.translation()(2) =  - dimension_(2) / 2;
  transform.linear() = Eigen::Matrix3d::Identity();

  contact->setTransform(transform);
  return contact;
}



}
