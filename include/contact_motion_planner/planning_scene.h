#ifndef PLANNING_SCENE_H
#define PLANNING_SCENE_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

#include "contact_motion_planner/contact_model/contact_model.h"
#include "contact_motion_planner/fcl_eigen_utils.h"

namespace suhan_contact_planner
{

class PlanningScene
{
  ContactModelPtr target_object_; ///< Object we want to check whether it collide with any env.

  std::vector< std::shared_ptr<fcl::ShapeBase> > objects_; ///< Environment objects (should not be changed)
  std::vector< fcl::Transform3f > transforms_;
public:
  bool isPossible() const;  ///< return whether collision between objects is free
  inline void setTargetObject(const ContactModelPtr &target_object) { target_object_ = target_object; }
  void addSceneObject(const std::shared_ptr<fcl::ShapeBase> &shape, const Eigen::Isometry3d &transform);
  void addSceneObject(const std::shared_ptr<fcl::ShapeBase> &shape, const fcl::Transform3f &transform);
};


typedef std::shared_ptr<PlanningScene> PlanningScenePtr;

}
#endif
