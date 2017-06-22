#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_broadcaster");

  ros::NodeHandle n;

  tf2_ros::TransformBroadcaster tb;
  geometry_msgs::TransformStamped ts;

  ros::Rate r(1.0);
  
  while(ros::ok()){
	ts.header.stamp = ros::Time::now();
	ts.header.frame_id = "frame1";
	ts.child_frame_id = "frame2";
	
	tf2::Vector3 translation;
	translation.setValue(0, 0, 1.0);
	ts.transform.translation.x = translation.x();
	ts.transform.translation.y = translation.y();
	ts.transform.translation.z = translation.z();
	
	tf2::Quaternion rotation;
	rotation.setRPY(0, 0, 0);
	ts.transform.rotation.x = rotation.x();
	ts.transform.rotation.y = rotation.y();
	ts.transform.rotation.z = rotation.z();
	ts.transform.rotation.w = rotation.w();
	
	tb.sendTransform(ts);
	ROS_INFO("Transform Published");
	
	r.sleep();
  }
  return 0;
}
