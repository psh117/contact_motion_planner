#include "contact_motion_planner/contact/contact.h"

namespace suhan_contact_planner
{


Contact::Contact(ContactState contact_state, ContactRelation contact_relation) :
  contact_relation_(contact_relation), contact_state_(contact_state)
{}

void Contact::printContactState()
{
  std::cout << "[Contact State] \n" <<
               "Transform: \n" <<
               transform_.matrix() << std::endl <<
               "Force: \n" <<
               contact_force_torque_.transpose() << std::endl;
}

}
