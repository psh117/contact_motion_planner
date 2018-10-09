#include "contact_motion_planner/contact_planner.h"
#include "contact_motion_planner/contact_model_graph.h"
#include "contact_motion_planner/contact_model/box_contact_model.h"

#include "contact_motion_planner/planning_scene.h"

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

void getBoxMarker(visualization_msgs::Marker& marker1, std::string frame_name, float r, float g, float b)
{
  marker1.header.frame_id = frame_name;
  marker1.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker1.  This serves to create a unique ID
  // Any marker1 sent with the same namespace and id will overwrite the old one
  marker1.ns = frame_name;
  marker1.id = 0;

  // Set the marker1 type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker1.type = visualization_msgs::Marker::CUBE;

  // Set the marker1 action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker1.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker1.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker1.pose.position.x = 0;
  marker1.pose.position.y = 0;
  marker1.pose.position.z = 0;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;

  // Set the scale of the marker1 -- 1x1x1 here means 1m on a side
  marker1.scale.x = 0.1;
  marker1.scale.y = 0.4;
  marker1.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker1.color.r = r;
  marker1.color.g = g;
  marker1.color.b = b;
  marker1.color.a = 1.0;

  marker1.lifetime = ros::Duration();
}

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
    dim << 0.1, 0.4, 0.2;
    shared_ptr<BoxContactModel> model = make_shared<BoxContactModel>("box", dim);
    Eigen::Affine3d start_transform;
    start_transform.linear() = Eigen::Matrix3d::Identity();
    start_transform.translation() << 0.0, 0.0, 0.1;
    cout << "before operation" << endl <<
            start_transform.matrix() << endl;
    model->setTransform(start_transform);
    model->setContactEnvironment(model->getBottomContact());
    //model->operate(ContactModel::OperationDirection::DIR_YAW, 0.10, -30 * 3.141592/180);
    model->operate(ContactModel::OperationDirection::DIR_ROLL, 0.10, -30 * 3.141592/180);
    model->operate(ContactModel::OperationDirection::DIR_PITCH, 0.10, -30 * 3.141592/180);
    cout << "after operation" << endl <<
            model->getTransform().matrix() << endl;

    model->operate(ContactModel::OperationDirection::DIR_PITCH, 0.10, -30 * 3.141592/180);
    model->operate(ContactModel::OperationDirection::DIR_ROLL, 0.10, -30 * 3.141592/180);
    cout << "after operation" << endl <<
            model->getTransform().matrix() << endl;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped before_operation_transform = tf2::eigenToTransform(start_transform);
    geometry_msgs::TransformStamped after_operation_transform = tf2::eigenToTransform(model->getTransform());

    before_operation_transform.header.stamp = ros::Time::now();
    before_operation_transform.header.frame_id = "world";
    before_operation_transform.child_frame_id = "before_operation";
    after_operation_transform.header.stamp = ros::Time::now();
    after_operation_transform.header.frame_id = "world";
    after_operation_transform.child_frame_id = "after_operation";
    static_broadcaster.sendTransform(before_operation_transform);
    static_broadcaster.sendTransform(after_operation_transform);

  }
  // Set our initial shape type to be a cube
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;
  getBoxMarker(marker1, "/before_operation",0,1,0);
  getBoxMarker(marker2, "/after_operation",1,0,0);
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.

  marker_array.markers.push_back(marker1);
  marker_array.markers.push_back(marker2);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  while (marker_pub.getNumSubscribers() < 1)
  {}

  marker_pub.publish(marker_array);


  // Graph generation module
  Eigen::Vector3d dim;
  dim << 0.1, 0.4, 0.2;
  shared_ptr<BoxContactModel> start = make_shared<BoxContactModel>("box", dim);
  start->setMass(1);
  start->setFriction(1.0);
  start->setSampleResolution(2,2);
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
  goal_transform.linear() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(90 * 3.141592/180., Eigen::Vector3d::UnitZ());
  start_transform.translation() << 0.0, 0.0, 0.1;
  goal_transform.translation() << .6, 0.5, .1;

  env_table1_transform.translation()    << .0, .0, -0.0251;
  env_obstacle1_transform.translation() << .25, .0, .25;
  env_table1_transform.linear() = Eigen::Matrix3d::Identity();
  env_obstacle1_transform.linear() = Eigen::Matrix3d::Identity();

  start->setTransform(start_transform);
  goal->setTransform(goal_transform);

  ContactModelGraph<BoxContactModel> g;
  PlanningScenePtr scene = make_shared<PlanningScene>();
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  scene->addSceneObject(env_table1, env_table1_transform);
  scene->addSceneObject(env_obstacle1, env_obstacle1_transform);
  g.setRobotDynamicsModel(robot_model);
  g.setPlanningScene(scene);
  g.setStart(start);
  g.setGoal(goal);
  cout << "Are you ready? : ";
  char h;
  //cin >> h;
  //if(h=='q') return 0;
  g.makeObjectPoseGraph();
  g.makeObjectContactGraph();
  g.printContactGraph();
  g.makeCombinationGraph();

  //start->cont

  // Test Contact Opt.


  return 0;
}
