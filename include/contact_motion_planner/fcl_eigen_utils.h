#ifndef FCL_EIGEN_UTILS_H
#define FCL_EIGEN_UTILS_H

#include <Eigen/Dense>
#include <fcl/shape/geometric_shapes.h>

namespace FCLEigenUtils
{

static void convertTransform(const fcl::Transform3f& fcl_input, Eigen::Isometry3d & eigen_output)
{
  auto &trans = fcl_input.getTranslation();
  auto &rot = fcl_input.getRotation();

  eigen_output.linear() << rot(0, 0), rot(0, 1), rot(0, 2),
                           rot(1, 0), rot(1, 1), rot(1, 2),
                           rot(2, 0), rot(2, 1), rot(2, 2);
  eigen_output.translation() << trans[0], trans[1], trans[2];
}

static void convertTransform(const Eigen::Isometry3d &eigen_input, fcl::Transform3f &fcl_output)
{
  fcl::Matrix3f rotation;
  fcl::Vec3f translation;

  auto &rot = eigen_input.linear();
  auto &trans = eigen_input.translation();

  rotation.setValue(rot(0,0), rot(0,1), rot(0,2),
                    rot(1,0), rot(1,1), rot(1,2),
                    rot(2,0), rot(2,1), rot(2,2));
  translation.setValue(trans(0), trans(1), trans(2));

  fcl_output.setTransform(rotation,translation);
}

}

#endif // FCL_EIGEN_UTILS_H
