#include "contact_motion_planner/planning_scene.h"
#include <ros/ros.h>

namespace suhan_contact_planner
{

bool PlanningScene::isPossible() const
{
  fcl::GJKSolver_libccd solver;
  fcl::Transform3f tr;

  for (size_t i=0; i<objects_.size(); i++)
  {
    fcl::ShapeBase& fd = *(target_object_->getFCLModel());
    fcl::ShapeBase& g = *(objects_[i]);

    fcl::Box * t = (fcl::Box*)&(*(target_object_->getFCLModel()));
    fcl::Box * t2 = (fcl::Box*)&(*(objects_[i]));
    auto & trans = target_object_->getFCLTransform().getTranslation();
    //    if(solver.shapeIntersect(*(target_object_->getFCLModel()),
    //                             target_object_->getFCLTransform(),
    //                             *(objects_[i]),
    //                             transforms_[i],
    //                             NULL))
    if(solver.shapeIntersect(*t,
                             target_object_->getFCLTransform(),
                             *t2,
                             transforms_[i],
                             NULL))
    {
      ROS_INFO("not possible %d", (int)i);
      return false; // not possible because there is a collision
    }
  }
  return true; // no collision
}


void PlanningScene::addSceneObject(const std::shared_ptr<fcl::ShapeBase> &shape, const Eigen::Affine3d &transform)
{
  fcl::Transform3f fcl_transform;
  FCLEigenUtils::convertTransform(transform, fcl_transform);

  addSceneObject(shape, fcl_transform);
}
void PlanningScene::addSceneObject(const std::shared_ptr<fcl::ShapeBase> &shape, const fcl::Transform3f &transform)
{
  objects_.push_back(shape);
  transforms_.push_back(transform);
}

}
