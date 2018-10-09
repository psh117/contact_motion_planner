#ifndef BOX_CONTACT_MODEL_H
#define BOX_CONTACT_MODEL_H

#include "contact_motion_planner/contact_model/contact_model.h"

namespace suhan_contact_planner
{

class BoxContactModel : public ContactModel
{
public:
  BoxContactModel(const Eigen::Vector3d &dimension);
  BoxContactModel(const std::string &name, const Eigen::Vector3d &dimension);

  size_t line_samples_per_side_;
  size_t point_samples_per_edge_;

  void setSampleResolution(size_t points_per_line, size_t lines_per_side)
  {
    point_samples_per_edge_ = points_per_line;
    line_samples_per_side_ = lines_per_side;
  }

  virtual void createContactSamples(std::vector<ContactPtr> &contact_samples) override;
  virtual bool operate(OperationDirection dir, double delta_x, double delta_orientation) override;
  virtual ContactPtr getBottomContact() override;
  // constraints? how?
protected:

private:
  Eigen::Vector3d dimension_; ///< width, length, height

};

}
#endif
