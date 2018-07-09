#ifndef CONTACT_PLANNER_H
#define CONTACT_PLANNER_H

#include <vector>
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>

using namespace std;
namespace suhan_contact_planner
{
class ObjectMotionDiscretizer
{

  double position_resolution;
  double orientation_resolution;

};

class Contact
{
public:
  enum ContactState {FACE_CONTACT, LINE_CONTACT, POINT_CONTAC, NO_CONTACT};

protected:
  const ContactState contact_state_;
private:
  Eigen::Vector3d position_;  ///< is set randomly or by discretization
  Eigen::Matrix3d rotation_;  ///< is set by model and position

};

class FaceContact : public Contact
{
  FaceContact() : contact_state_ {FaceContact}
  {

  }
};

class ContactModel
{
public:

protected:
  tf2::Transform transform_;
  vector < Contact > contact_candidate_;

  int sample_; ///< number of samples


};
class Wrench
{

};
class BoxContactModel : public ContactModel
{
public:

private:
  Eigen::Vector3d dimension_; ///< width, length, height
};

class ContactConfiguration
{
  Eigen::Matrix<double, 6, 1> wrench_; ///< Contact wrench
  tf2::Transform transform_;  ///< Contact position
};

class ContactPlanner
{
  // Pose graph

};

}
#endif
