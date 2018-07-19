#ifndef PLANNING_SCENE_H
#define PLANNING_SCENE_H

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include "contact_motion_planner/contact_model/contact_model.h"

namespace suhan_contact_planner
{

class PlanningScene
{
  ContactModelPtr target_object_; ///< Object we want to check whether it collide with any env.
  std::vector< std::shared_ptr<fcl::ShapeBase> > objects_; ///< Environment objects (should not be changed)

public:
  bool isPossible() const;  ///< return whether collision between objects is free
  inline void setTargetObject(const ContactModelPtr &target_object) { target_object_ = target_object; }
};


typedef std::shared_ptr<PlanningScene> PlanningScenePtr;

}
#endif
