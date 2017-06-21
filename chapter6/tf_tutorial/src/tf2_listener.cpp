#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ros::Rate r(10.0);
  
  while(ros::ok()){
	geometry_msgs::TransformStamped transform;
	try{
	  transform = tfBuffer.lookupTransform("frame1", "frame2", ros::Time(0));
	}
	catch (tf::TransformException &ex){
	  ROS_ERROR("%s", ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	}

	ROS_INFO("\n=== Got Transform ===\n"
			 " x : %f\n y : %f\n z : %f\n"
			 " x : %f\n y : %f\n z : %f\n w : %f",
			 transform.transform.translation.x,
			 transform.transform.translation.y,
			 transform.transform.translation.z,
			 transform.transform.rotation.x,
			 transform.transform.rotation.y,
			 transform.transform.rotation.z,
			 transform.transform.rotation.w);
	
	r.sleep();
  }
  return 0;
}
