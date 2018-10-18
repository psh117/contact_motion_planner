#include <contact_motion_planner/contact_model/visualization/contact_model_visualization.h>

namespace suhan_contact_planner
{
Eigen::Vector3d hsv2rgb(const Eigen::Vector3d& in)
{
    double      hh, p, q, t, ff;
    long        i;
    Eigen::Vector3d out;
    double h, s, v;
    h = in(0);
    s = in(1);
    v = in(2);

    if(s <= 0.0) {       // < is bogus, just shuts up warnings
        out(0) = v;
        out(1) = v;
        out(2) = v;
        return out;
    }
    hh = h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        out(0) = v;
        out(1) = t;
        out(2) = p;
        break;
    case 1:
        out(0) = q;
        out(1) = v;
        out(2) = p;
        break;
    case 2:
        out(0) = p;
        out(1) = v;
        out(2) = t;
        break;

    case 3:
        out(0) = p;
        out(1) = q;
        out(2) = v;
        break;
    case 4:
        out(0) = t;
        out(1) = p;
        out(2) = v;
        break;
    case 5:
    default:
        out(0) = v;
        out(1) = p;
        out(2) = q;
        break;
    }
    return out;
}
void ContactModelVisualization::frameAssignment()
{
  int number = 0;
  for (auto & node : contact_list_)
  {
    geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(node->getTransform());

    std::string box_marker_name = "box_model_" + std::to_string(number);
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "world";
    transform_msg.child_frame_id = box_marker_name;
    static_broadcaster_.sendTransform(transform_msg);
    int contact_number = 0;
    for (auto & contact : node->getContactRobot())
    {
      geometry_msgs::TransformStamped transform_msg_c = tf2::eigenToTransform(contact->getContactTransform());

      transform_msg_c.header.stamp = ros::Time::now();
      std::string box_contact_marker_name = box_marker_name + "_contact_" + std::to_string(contact_number);
      transform_msg_c.header.frame_id = box_marker_name;
      transform_msg_c.child_frame_id = box_contact_marker_name;
      static_broadcaster_.sendTransform(transform_msg_c);
      contact_number++;

    }

    number++;
  }
}
void ContactModelVisualization::makeMarkerArray()
{
  char h = 0;
  marker_array_.markers.clear();
  while (ros::ok())
  {
    double angle = 0;
    int number = 0;
    for (auto & node : contact_list_)
    {
      visualization_msgs::Marker marker;
      std::string box_marker_name = "box_model_" + std::to_string(number);
      getBoxMarker(marker, box_marker_name, hsv2rgb(Eigen::Vector3d(angle,1.,1.)));
      marker_array_.markers.push_back(marker);
      int contact_number = 0;
      for (auto & contact : node->getContactRobot())
      {
        Eigen::Vector3d force = contact->getContactForceTorque().head<3>();
        force = contact->getContactTransform().linear().transpose() * force;
        visualization_msgs::Marker marker_c;
        std::string box_contact_marker_name = box_marker_name + "_contact_" + std::to_string(contact_number);
        getArrowMarker(marker_c, box_contact_marker_name, hsv2rgb(Eigen::Vector3d(angle,1.,1.)), force);
        marker_array_.markers.push_back(marker_c);
        contact_number++;
      }
      angle += 360. / contact_list_.size();
      number++;

      marker_pub_.publish(marker_array_);
      ROS_INFO("press any key to show");
      std::cin >> h;
      if (h=='q') break;
      marker_array_.markers.clear();
    }
    if( h== 'q' ) break;
  }
}

void ContactModelVisualization::getBoxMarker(visualization_msgs::Marker& marker,
                                             const std::string &frame_name, Eigen::Vector3d color)
{
  marker.header.frame_id = frame_name;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = frame_name;
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  //tf2::doTransform(marker.pose, marker.pose, t);

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.4;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
}

void ContactModelVisualization::getArrowMarker(visualization_msgs::Marker& marker,
                  const std::string &frame_name, Eigen::Vector3d color, const Eigen::Matrix<double, 3, 1> &force)
{
  marker.header.frame_id = frame_name;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = frame_name;
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  geometry_msgs::Point a, b;
  a.x = 0; a.y =0; a.z = 0;
  b.x = -force(0) * 0.01;
  b.y = -force(1) * 0.01;
  b.z = -force(2) * 0.01;
  marker.points.push_back(b);
  marker.points.push_back(a);

  marker.scale.x = 0.007;
  marker.scale.y = 0.025;
  marker.scale.z = 0;
      /*
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.07;
*/
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
}


} // namespace suhan_contact_planner
