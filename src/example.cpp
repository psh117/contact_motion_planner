#include "contact_motion_planner/contact_planner.h"
#include "contact_motion_planner/contact_model_graph.h"
#include "contact_motion_planner/contact_model/box_contact_model.h"

#include "contact_motion_planner/planning_scene.h"

#include "contact_motion_planner/fcl_eigen_utils.h"
#include "contact_motion_planner/robot_dynamics/dexterous_robot_model.h"

#include <ros/ros.h>


// Declare a test
using namespace std;
using namespace suhan_contact_planner;
int main()
{

  Eigen::Vector3d dim;
  dim << 0.2, 0.2, 0.2;
  shared_ptr<BoxContactModel> start = make_shared<BoxContactModel>("box", dim);
  shared_ptr<BoxContactModel> goal = make_shared<BoxContactModel>("box", dim);
  shared_ptr<fcl::Box> env_table1 = make_shared<fcl::Box>(0.7, 0.7, 0.05);
  shared_ptr<fcl::Box> env_obstacle1 = make_shared<fcl::Box>(0.1, 0.1, 0.5);
  Eigen::Isometry3d env_table1_transform;
  Eigen::Isometry3d env_obstacle1_transform;
  Eigen::Isometry3d start_transform;
  Eigen::Isometry3d goal_transform;

  start_transform.linear() = Eigen::Matrix3d::Identity();
  goal_transform.linear() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(90 * 3.141592/180., Eigen::Vector3d::UnitZ());
  start_transform.translation() << 0.0, 0.0, 0.0;
  goal_transform.translation() << .6, 0.5, .0;

  env_table1_transform.translation()    << .0, .0, -0.025;
  env_obstacle1_transform.translation() << -.25, .1, .0;
  env_table1_transform.linear() = Eigen::Matrix3d::Identity();
  env_obstacle1_transform.linear() = Eigen::Matrix3d::Identity();

  start->setTransform(start_transform);
  goal->setTransform(goal_transform);

  ContactModelGraph<BoxContactModel> g;
  PlanningScenePtr scene = make_shared<PlanningScene>();
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  //scene->addSceneObject(env_table1, env_table1_transform);
  //scene->addSceneObject(env_obstacle1, env_obstacle1_transform);
  g.setRobotDynamicsModel(robot_model);
  g.setPlanningScene(scene);
  g.setStart(start);
  g.setGoal(goal);
  cout << "Are you ready? : ";
  char h;
  cin >> h;
  g.makeGraph();
  return 0;
}
