#include "contact_motion_planner/contact_model/box_contact_model.h"


namespace suhan_contact_planner
{


BoxContactModel::BoxContactModel(const std::string &name, const Eigen::Vector3d &dimension) :
  ContactModel(name), dimension_(dimension)
{
  fcl_model_ = std::make_shared<fcl::Box>(dimension_(0),
                                          dimension_(1),
                                          dimension_(2));
}
BoxContactModel::BoxContactModel(const Eigen::Vector3d &dimension) : dimension_(dimension)
{
  BoxContactModel("", dimension);
}


}
