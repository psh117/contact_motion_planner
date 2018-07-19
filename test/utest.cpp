// Bring in my package's API, which is what I'm testing
#include "contact_motion_planner/contact_planner.h"
#include "contact_motion_planner/contact_model_graph.h"
#include "contact_motion_planner/contact_model/box_contact_model.h"

#include <ros/ros.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
using namespace std;
using namespace suhan_contact_planner;
TEST(ContactModelGraphSuite, testCase1)
{
  Eigen::Vector3d dim;
  dim << 1.0, 1.0, 1.0;
  shared_ptr<BoxContactModel> start( new BoxContactModel("box", dim) );
  shared_ptr<BoxContactModel> goal( new BoxContactModel("box", dim) );
  ContactModelGraph<BoxContactModel> g;
  g.makeGraph();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "contact_motion_planner_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
