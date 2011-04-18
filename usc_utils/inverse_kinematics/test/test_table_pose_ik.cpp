#include <ros/ros.h>
#include <robot_info/robot_info_init.h>
#include <inverse_kinematics/inverse_kinematics.h>
#include <visualization_msgs/Marker.h>

using namespace inverse_kinematics;

visualization_msgs::Marker markers_;
std_msgs::ColorRGBA good_color_;
std_msgs::ColorRGBA bad_color_;
ros::Publisher marker_pub_;

void initMarkers()
{
  markers_.header.frame_id="/BASE";
  markers_.id=0;
  markers_.ns="test_table_pose_ik";
  markers_.type=markers_.CUBE_LIST;
  markers_.action=markers_.ADD;
  markers_.scale.x=0.025;
  markers_.scale.y=0.025;
  markers_.scale.z=0.025;
  markers_.pose.orientation.w=1.0;
  markers_.pose.orientation.x=0.0;
  markers_.pose.orientation.y=0.0;
  markers_.pose.orientation.z=0.0;
  markers_.pose.position.x=0.0;
  markers_.pose.position.y=0.0;
  markers_.pose.position.z=0.0;
  markers_.lifetime=ros::Duration(0.0);

  good_color_.a=0.5;
  good_color_.r=0.0;
  good_color_.g=1.0;
  good_color_.b=0.0;
  bad_color_.a=0.5;
  bad_color_.r=1.0;
  bad_color_.g=0.0;
  bad_color_.b=0.0;

  markers_.color = good_color_;

}

void addMarker(double x, double y, double z, std_msgs::ColorRGBA& color)
{
  markers_.header.stamp = ros::Time::now();
  geometry_msgs::Point point;
  point.x=x;
  point.y=y;
  point.z=z;
  markers_.points.push_back(point);
  markers_.colors.push_back(color);
  marker_pub_.publish(markers_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_table_ik_poses");
  robot_info::init();

  ros::NodeHandle node_handle;
  InverseKinematics ik("RIGHT_ARM");
  marker_pub_=node_handle.advertise<visualization_msgs::Marker>("/inverse_kinematics/test_table_ik_poses_markers", 100, true);
  initMarkers();

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  geometry_msgs::Pose pose;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 1.0;
  pose.orientation.z = 0.0;

  std::vector<double> joint_angles;

  for (double x=-0.5; x<=1.2; x+=0.1)
  {
    for (double y=0.2; y<=1.2; y+=0.1)
    {
      for (double z=0.85; z<=1.1; z+=0.05)
      {
        pose.position.x=x;
        pose.position.y=y;
        pose.position.z=z;

        bool success = ik.ik(pose, joint_angles);

        if (success)
          addMarker(x,y,z, good_color_);
        else
          addMarker(x,y,z, bad_color_);

        ros::spinOnce();

        if (!node_handle.ok())
          return 0;
      }
    }
  }

}
