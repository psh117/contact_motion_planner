#include "contact_motion_planner/contact_planner.h"
#include "contact_motion_planner/contact_model_graph.h"
#include "contact_motion_planner/contact_model/box_contact_model.h"

#include "contact_motion_planner/planning_scene.h"
#include <contact_motion_planner/contact_model/visualization/contact_model_visualization.h>

#include "contact_motion_planner/fcl_eigen_utils.h"
#include "contact_motion_planner/robot_dynamics/dexterous_robot_model.h"

#include "contact_motion_planner/solver/contact_optimization.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

// Declare a test
using namespace std;
using namespace suhan_contact_planner;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "suhan_contact_planner_ex");
  ros::NodeHandle nh;

  // ContactModel module
  {
    Eigen::Vector3d dim;
    dim << 0.3, 0.4, 0.2;
    shared_ptr<BoxContactModel> model = make_shared<BoxContactModel>("box", dim);
    Eigen::Affine3d start_transform;
    start_transform.linear() = Eigen::Matrix3d::Identity();
    start_transform.translation() << 0.0, 0.0, 0.1;
    cout << "before operation" << endl <<
            start_transform.matrix() << endl;
    model->setTransform(start_transform);
    model->setContactEnvironment(model->getBottomContact());
    //model->operate(ContactModel::OperationDirection::DIR_YAW, 0.10, -30 * 3.141592/180);
    model->operate(ContactModel::OperationDirection::DIR_ROLL, 0.10, 30 * M_PI/180.);
    cout << "after operation" << endl <<
            model->getTransform().matrix() << endl;

    //model->operate(ContactModel::OperationDirection::DIR_PITCH, 0.10, -30 * 3.141592/180);
    //model->operate(ContactModel::OperationDirection::DIR_ROLL, 0.10, -30 * 3.141592/180);
    cout << "after operation" << endl <<
            model->getTransform().matrix() << endl;
  }
  // Set our initial shape type to be a cube
  // Graph generation module
  Eigen::Vector3d dim;
  dim << 0.1, 0.4, 0.2;
  shared_ptr<BoxContactModel> start = make_shared<BoxContactModel>("box", dim);
  start->setMass(1);
  start->setFriction(0.2);
  start->setSampleResolution(3,0);
  start->setContactEnvironment(start->getBottomContact());
  shared_ptr<BoxContactModel> goal = make_shared<BoxContactModel>(*start);
  shared_ptr<fcl::Box> env_table1 = make_shared<fcl::Box>(0.7, 0.7, 0.05);
  shared_ptr<fcl::Box> env_obstacle1 = make_shared<fcl::Box>(0.1, 0.1, 0.5);
  Eigen::Affine3d env_table1_transform;
  Eigen::Affine3d env_obstacle1_transform;
  Eigen::Affine3d start_transform;
  Eigen::Affine3d goal_transform;

  ROS_INFO("START");

  std::vector<ContactPtr> samples;
  start->createContactSamples(samples);

  ROS_INFO("START");
  for(const auto& sample : samples)
  {
    std::cout << sample->getContactTransform().translation().transpose() << std::endl;
  }
  start_transform.linear() = Eigen::Matrix3d::Identity();
  goal_transform.linear() <<  1,          0,          0,
                              0,   0.866025,       -0.5,
                              0,        0.5,   0.866025;
  //goal_transform.linear().setIdentity();
  //goal_transform.linear().setIdentity() * Eigen::AngleAxisd(45 * 3.141592/180., Eigen::Vector3d::UnitZ());
  start_transform.translation() << 0.6, 0.0, 0.85;
  goal_transform.translation() << 0.6, -0.0767949, 0.186603 + .2 + .75;
  //goal_transform.translation() << .0, .0, .4;

  env_table1_transform.translation()    << 0.6, .0, 0.75 - 0.05/2. -0.0001 ;
  env_obstacle1_transform.translation() << .25, .0, .25;
  env_table1_transform.linear() = Eigen::Matrix3d::Identity();
  env_obstacle1_transform.linear() = Eigen::Matrix3d::Identity();

  start->setTransform(start_transform);
  goal->setTransform(goal_transform);


  ContactModelGraph<BoxContactModel> g;
  PlanningScenePtr scene = make_shared<PlanningScene>();
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  scene->addSceneObject(env_table1, env_table1_transform);
  //scene->addSceneObject(env_obstacle1, env_obstacle1_transform);
  g.setRobotDynamicsModel(robot_model);
  g.setPlanningScene(scene);
  g.setStart(start);
  g.setGoal(goal);


  cout << "Are you ready? : ";
  char h;
  cin >> h;
  if(h=='q') return 0;


  ROS_INFO("makeObjectPoseGraph");
  g.makeObjectPoseGraph();
  ROS_INFO("makeObjectContactGraph");
  g.makeObjectContactGraph();
  //g.printContactGraph();
  ROS_INFO("makeCombinationGraph");
  g.makeCombinationGraph();
  //g.printTotalGraph();
  ROS_INFO("searchGoal");
  //std::vector<ContactModelPtr> result;

  std::vector<ContactModelPtr> result;
//  result.push_back(start);
  //result.push_back(goal);

  g.searchGoal(result);

  ContactModelVisualization viz(nh, result);

  ros::spin();
  //ContactModelVisualization viz(nh, result);

  //start->cont

  // Test Contact Opt.


  return 0;
}
