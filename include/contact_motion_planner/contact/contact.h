#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace suhan_contact_planner
{

class Contact
{
public:
  enum class ContactRelation {CONTACT_OBJ_ENV, CONTACT_OBJ_ROBOT};
  enum class ContactState {CONTACT_FACE, CONTACT_LINE, CONTACT_POINT};

  Contact(ContactState contact_state = ContactState::CONTACT_POINT, 
          ContactRelation contact_relation = ContactRelation::CONTACT_OBJ_ROBOT) ;
  inline const ContactState getContactState() const {return contact_state_; }
  inline const Eigen::Affine3d & getContactTransform() { return transform_; }

  void setTransform(const Eigen::Affine3d& transform) { transform_ = transform; }
  void setContactState(ContactState state) { contact_state_ = state; }

  void printContactState();

protected:
  ContactRelation contact_relation_;
  ContactState contact_state_;

private:
  // position & rotation with regard to object frame
  Eigen::Affine3d transform_; ///< Contact position transform w.r.t object frame (\f$objH_c\f$)
  //Eigen::Vector3d position_;  ///< is set randomly or by discretization
  //Eigen::Matrix3d rotation_;  ///< is set by model and position


  Eigen::Vector3d contact_force_;
  Eigen::Vector3d contact_torque_;

};

typedef std::shared_ptr<Contact> ContactPtr;


}
