#ifndef CONTACT_MODEL_H
#define CONTACT_MODEL_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>

#include "contact_motion_planner/contact/contact.h"
#include "contact_motion_planner/fcl_eigen_utils.h"

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
   * @brief createContactSamples
   */
  virtual void createContactSamples(std::vector<ContactPtr> &contact_samples) = 0;

  /**
   * @brief operation
   * @param dir Direction we want to move
   * @param delta
   * @return Whether it is possible operation.
   */
  virtual bool operate(OperationDirection dir, double delta_x, double delta_orientation);

  /**
   * @brief isSamePose
   * @param model An model to be compared
   * @param threshold_x Linear threshold
   * @param threshold_orientation Orientation threshold
   * @return
   */
  bool isSamePose(const ContactModel& model, double threshold_x, double threshold_orientation) const;


  inline const Eigen::Vector3d getPosition() const {return transform_.translation(); }
  void setTransform(const Eigen::Isometry3d &transform) { transform_ = transform; updateFCLModel(); }
  inline const Eigen::Isometry3d getTransform() const { return transform_; }

  inline std::shared_ptr< fcl::ShapeBase >& getFCLModel() { return fcl_model_; }
  inline fcl::Transform3f& getFCLTransform() { return fcl_transform_; }

  inline size_t getContactNumberRobot() { return contact_robot_.size(); }
  inline size_t getContactNumberEnvironment() { return contact_environment_.size(); }
  inline size_t getContactNumber() { return getContactNumberEnvironment() + getContactNumberRobot(); }

  const std::vector<ContactPtr>& getContactRobot() const { return contact_robot_; }
  const std::vector<ContactPtr>& getContactEnvironment() const { return contact_environment_; }

  // const std::vector < ContactPtr > & getContactSamples() { return contact_samples_; }
  inline double getMass() { return mass_; }
  inline double getFriction() { return friction_; }
  void setFriction(double friction) { friction_ = friction; }

protected:
  std::string name_;
  Eigen::Isometry3d transform_;
  Eigen::Vector3d centor_of_mass_ {0, 0, 0};

  // std::vector < ContactPtr > line_contact_samples_;
  // std::vector < ContactPtr > point_contact_samples_;
  //std::vector < ContactPtr > contact_samples_;
  std::vector < ContactPtr > contact_robot_;
  std::vector < ContactPtr > contact_environment_;
  std::shared_ptr< fcl::ShapeBase > fcl_model_;
  fcl::Transform3f fcl_transform_;


  virtual void updateFCLModel();

  double mass_;
  double friction_;

  //virtual void sample() = 0;
};

typedef std::shared_ptr<ContactModel> ContactModelPtr;

}

#endif
