#ifndef BOX_CONTACT_MODEL_H
#define BOX_CONTACT_MODEL_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include "contact_motion_planner/contact_model/contact_model.h"

namespace suhan_contact_planner
{

class BoxContactModel : public ContactModel
{
public:
  BoxContactModel(const Eigen::Vector3d &dimension);
  BoxContactModel(const std::string &name, const Eigen::Vector3d &dimension);

  // constraints? how?
protected:
  //virtual void sample();
private:
  Eigen::Vector3d dimension_; ///< width, length, height

};

}
#endif
