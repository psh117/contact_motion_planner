#ifndef FACE_CONTACT_H
#define FACE_CONTACT_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

#include "contact_motion_planner/contact/contact.h"

namespace suhan_contact_planner
{


class FaceContact : public Contact
{
  FaceContact(ContactRelation contact_relation) : Contact(contact_relation, CONTACT_FACE) // Rectangle? Assume...
  {
    fcl::Box g;
  }
};


}
#endif
