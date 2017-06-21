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
	tf2::Vector3 v;
	v.setValue(0, 0, 1.0);
	ts.transform.translation.x = v.x();
	ts.transform.translation.y = v.y();
	ts.transform.translation.z = v.z();
	tf2::Quaternion q;
	q.setRPY(0, 0, 0);
	ts.transform.rotation.x = q.x();
	ts.transform.rotation.y = q.y();
	ts.transform.rotation.z = q.z();
	ts.transform.rotation.w = q.w();

	tb.sendTransform(ts);
	
	r.sleep();
  }
  return 0;
}
