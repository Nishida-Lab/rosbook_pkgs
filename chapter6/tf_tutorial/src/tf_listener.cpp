#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle n;

  tf::TransformListener listner;
  ros::Rate r(10.0);
  
  while(ros::ok()){
	tf::StampedTransform transform;
	try{
	  listner.lookupTransform("/frame1", "/frame2",
							  ros::Time(0), transform);
	}
	catch (tf::TransformException &ex){
	  ROS_ERROR("%s", ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	}

	ROS_INFO("\n=== Got Transform ===\n"
			 " x : %f\n y : %f\n z : %f\n"
			 " x : %f\n y : %f\n z : %f\n w : %f",
			 transform.getOrigin().x(),
			 transform.getOrigin().y(),
			 transform.getOrigin().z(),
			 transform.getRotation().x(),
			 transform.getRotation().y(),
			 transform.getRotation().z(),
			 transform.getRotation().w());
	
	r.sleep();
  }
  return 0;
}
