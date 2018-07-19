#ifndef FACE_CONTACT_H
#define FACE_CONTACT_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "contact_motion_planner/contact/contact.h"

namespace suhan_contact_planner
{


class FaceContact : public Contact
{
  FaceContact(ContactRelation contact_relation) : Contact(contact_relation, CONTACT_FACE) // Rectangle? Assume...
  {
  }
};


}
#endif
