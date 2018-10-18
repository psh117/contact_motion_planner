#pragma once

#include "contact_motion_planner/contact_model/contact_model.h"
#include "contact_motion_planner/contact_model_graph.h"

namespace suhan_contact_planner
{

class ContactPlanner
{
  // Pose graph
  ContactModelPtr contact_model_;

  void plan();
};

}
