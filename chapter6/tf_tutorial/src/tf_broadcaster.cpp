#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle n;

  tf::TransformBroadcaster tb;
  geometry_msgs::TransformStamped ts;

  ros::Rate r(1.0);
  
  while(ros::ok()){
	ts.header.stamp = ros::Time::now();
	ts.header.frame_id = "frame1";
	ts.child_frame_id = "frame2";
	tf::Vector3 v;
	v.setValue(0, 0, 1.0);
	ts.transform.translation.x = v.x();
	ts.transform.translation.y = v.y();
	ts.transform.translation.z = v.z();
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	ts.transform.rotation.x = q.x();
	ts.transform.rotation.y = q.y();
	ts.transform.rotation.z = q.z();
	ts.transform.rotation.w = q.w();

	tb.sendTransform(ts);
	ROS_INFO("Transform Published");
	
	r.sleep();
  }
  return 0;
}
