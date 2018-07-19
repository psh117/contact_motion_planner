#ifndef CONTACT_MODEL_H
#define CONTACT_MODEL_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

#include "contact_motion_planner/contact/face_contact.h"

namespace suhan_contact_planner
{

/**
 * @brief The ContactModel class
 */
class ContactModel
{
public:
  ContactModel() : name_("") {}
  ContactModel(std::string name) : name_(name) {}

  enum OperationDirection : int {DIR_X=0, DIR_Y, DIR_Z, DIR_YAW, DIR_PITCH, DIR_ROLL};

  void contactWrenchOptimize();

  /**
   * @brief operation
   * @param dir Direction we want to move
   * @param delta
   * @return Whether it is possible operation.
   */
  bool operate(OperationDirection dir, double delta);
  bool isSamePose(const ContactModel& model, double threshold) const;

  inline const Eigen::Vector3d getPosition() const {return transform_.translation(); }
  inline const Eigen::Isometry3d getTransform() const { return transform_; }

protected:
  std::string name_;
  Eigen::Isometry3d transform_;
  std::vector < ContactPtr > contact_candidate_;
  std::vector < ContactPtr > contact_environment_;
  std::shared_ptr< fcl::ShapeBase > fcl_model_;



  int sample_; ///< number of samples

  double mass_;
  double friction_;

  //virtual void sample() = 0;
};

typedef std::shared_ptr<ContactModel> ContactModelPtr;

}

#endif
