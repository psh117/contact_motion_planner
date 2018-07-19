#ifndef CONTACT_H
#define CONTACT_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace suhan_contact_planner
{

class Contact
{
public:
  enum ContactRelation {CONTACT_OBJ_ENV, CONTACT_OBJ_ROBOT};
  enum ContactState {CONTACT_FACE, CONTACT_LINE, CONTACT_POINT};
  // Contact() {}
  Contact(ContactRelation contact_relation, ContactState contact_state);
  inline const ContactState getContactState() const {return contact_state_; }

protected:
  ContactRelation contact_relation_;
  ContactState contact_state_;

private:
  // position & rotation with regard to object frame
  Eigen::Vector3d position_;  ///< is set randomly or by discretization
  Eigen::Matrix3d rotation_;  ///< is set by model and position


  Eigen::Vector3d contact_force_;
  Eigen::Vector3d contact_torque_;

};

typedef std::shared_ptr<Contact> ContactPtr;


}
#endif
