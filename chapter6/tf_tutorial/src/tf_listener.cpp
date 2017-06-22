#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
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
	  listner.lookupTransform("frame1", "frame2",
							  ros::Time(0), transform);
	}
	catch (tf::TransformException &ex){
	  ROS_ERROR("%s", ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	}

	tf::Vector3 translation;
	translation.setValue(transform.getOrigin().x(),
						 transform.getOrigin().y(),
						 transform.getOrigin().z());
	
	tf::Quaternion rotation;
	rotation.setValue(transform.getRotation().x(),
					  transform.getRotation().y(),
					  transform.getRotation().z(),
					  transform.getRotation().w());
	
	tf::Matrix3x3 m(rotation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	ROS_INFO("\n=== Got Transform ===\n"
			 " Translation\n"
			 " x : %f\n y : %f\n z : %f\n"
			 " Quaternion\n"
			 " x : %f\n y : %f\n z : %f\n w : %f\n"
			 " RPY\n"
			 " R : %f\n P : %f\n Y : %f",
			 translation.x(), translation.y(), translation.z(),
			 rotation.x(), rotation.y(), rotation.z(), rotation.w(),
			 roll, pitch, yaw);
	
	r.sleep();
  }
  return 0;
}
