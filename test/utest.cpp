// Bring in my package's API, which is what I'm testing
#include "contact_motion_planner/contact_planner.h"
#include "contact_motion_planner/contact_model_graph.h"
#include "contact_motion_planner/contact_model/box_contact_model.h"

#include "contact_motion_planner/planning_scene.h"

#include "contact_motion_planner/fcl_eigen_utils.h"
#include "contact_motion_planner/robot_dynamics/dexterous_robot_model.h"

#include <ros/ros.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
using namespace std;
using namespace suhan_contact_planner;

TEST(EigenFCLUtilsTestSuite, testCase1)
{
  Eigen::Isometry3d transform_eigen;
  fcl::Transform3f transform_fcl;

  Eigen::Vector3d trans;
  trans << 1.0, 2.0, 3.0;

  transform_eigen.linear() = Eigen::Matrix3d::Identity();
  transform_eigen.translation() = trans;

  FCLEigenUtils::convertTransform(transform_eigen, transform_fcl);

  EXPECT_EQ(transform_fcl.getTranslation()[0], 1.0);
  EXPECT_EQ(transform_fcl.getTranslation()[1], 2.0);
  EXPECT_EQ(transform_fcl.getTranslation()[2], 3.0);
}


TEST(PlanningSceneSuite, testCase1)
{
  Eigen::Vector3d dim;
  Eigen::Isometry3d transform;
  Eigen::Isometry3d transform_env1;
  transform.linear() = Eigen::Matrix3d::Identity();
  transform.translation() << 0, 0, 0;
  transform_env1.linear() = Eigen::Matrix3d::Identity();
  transform_env1.translation() << 0, 0, 0;
  dim << .5, .5, .5;

  std::shared_ptr<fcl::Box> env_box1 = std::make_shared<fcl::Box>(0.2, 0.2, 0.2);

  shared_ptr<BoxContactModel> box1( new BoxContactModel("box", dim) );
  box1->setTransform(transform);
  PlanningScene scene;
  scene.setTargetObject(box1);
  scene.addSceneObject(env_box1,transform_env1);

  EXPECT_FALSE(scene.isPossible());

  transform.translation() << 0, 0, 0.351;
  box1->setTransform(transform);

  EXPECT_TRUE(scene.isPossible());
}

TEST(ContactModelGraphSuite, testCase1)
{
  Eigen::Vector3d dim;
  dim << 0.2, 0.2, 0.2;
  shared_ptr<BoxContactModel> start( new BoxContactModel("box", dim) );
  shared_ptr<BoxContactModel> goal( new BoxContactModel("box", dim) );
  shared_ptr<fcl::Box> env_table1 = make_shared<fcl::Box>(0.7, 0.7, 0.05);
  shared_ptr<fcl::Box> env_obstacle1 = make_shared<fcl::Box>(0.1, 0.1, 0.5);
  Eigen::Isometry3d env_table1_transform;
  Eigen::Isometry3d env_obstacle1_transform;
  Eigen::Isometry3d start_transform;
  Eigen::Isometry3d goal_transform;

  start_transform.linear() = Eigen::Matrix3d::Identity();
  goal_transform.linear() = Eigen::Matrix3d::Identity();
  start_transform.translation() << 0.0, 0.0, 0.0;
  goal_transform.translation() << .2, 0.2, .0;

  env_table1_transform.translation()    << .0, .0, -0.025;
  env_obstacle1_transform.translation() << -.25, .1, .0;
  env_table1_transform.linear() = Eigen::Matrix3d::Identity();
  env_obstacle1_transform.linear() = Eigen::Matrix3d::Identity();

  start->setTransform(start_transform);
  goal->setTransform(goal_transform);

  ContactModelGraph<BoxContactModel> g;
  PlanningScenePtr scene(new  PlanningScene);
  RobotDynamicsModelPtr robot_model(new DexterousRobotModel);
  //scene->addSceneObject(env_table1, env_table1_transform);
  //scene->addSceneObject(env_obstacle1, env_obstacle1_transform);
  g.setRobotDynamicsModel(robot_model);
  g.setPlanningScene(scene);
  g.setStart(start);
  g.setGoal(goal);
  g.makeGraph();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "contact_motion_planner_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
