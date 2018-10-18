#pragma once

#include <vector>
#include <contact_motion_planner/contact_model/contact_model.h>

#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
namespace suhan_contact_planner
{

/**
 * @brief hsv2rgb Convert from HSV to RGB
 * @param in (0): H, (1): S, (2): V
 * @return (0): Red, (1): Green, (2): Blue
 */
static Eigen::Vector3d hsv2rgb(const Eigen::Vector3d& in);


class ContactModelVisualization
{
public:
  ContactModelVisualization(ros::NodeHandle & nh, std::vector<ContactModelPtr> contact_list) :
    contact_list_(contact_list)
  {
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    while (marker_pub_.getNumSubscribers() < 1)
    {}
    frameAssignment();
    makeMarkerArray();
  }
private:
  void frameAssignment();
  void makeMarkerArray();

  void getBoxMarker(visualization_msgs::Marker& marker,
                    const std::string &frame_name, Eigen::Vector3d color);

  void getArrowMarker(visualization_msgs::Marker& marker,
                      const std::string &frame_name, Eigen::Vector3d color, const Eigen::Matrix<double, 3, 1> &force);

  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  std::vector<ContactModelPtr> contact_list_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};



} // namespace suhan_contact_planner
