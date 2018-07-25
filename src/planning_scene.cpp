#include "contact_motion_planner/planning_scene.h"



namespace suhan_contact_planner
{

bool PlanningScene::isPossible() const
{
  fcl::GJKSolver_libccd solver;
  fcl::Transform3f tr;

  for (size_t i=0; i<objects_.size(); i++)
  {
    if(solver.shapeIntersect(*target_object_->getFCLModel(),
                          target_object_->getFCLTransform(),
                          objects_[i],
                          transforms_[i]))
    {
      return false; // not possible because there is a collision
    }
  }
  return true; // no collision
}


void PlanningScene::addSceneObject(std::shared_ptr<fcl::ShapeBase> &shape, const Eigen::Isometry3d &transform)
{
  fcl::Transform3f fcl_transform;
  FCLEigenUtils::convertTransform(transform, fcl_transform);

  addSceneObject(shape, fcl_transform);
}
void PlanningScene::addSceneObject(std::shared_ptr<fcl::ShapeBase> &shape, const fcl::Transform3f &transform)
{
  objects_.push_back(shape);
  transforms_.push_back(transform);
}

}
