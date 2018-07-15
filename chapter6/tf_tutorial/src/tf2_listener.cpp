#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate r(10.0);

  while(ros::ok())
  {
    geometry_msgs::TransformStamped transform;

    try
    {
      transform = tfBuffer.lookupTransform("frame1", "frame2", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf2::Vector3 translation;
    translation.setValue(transform.transform.translation.x,
                         transform.transform.translation.y,
                         transform.transform.translation.z);

    tf2::Quaternion rotation;
    rotation.setValue(transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z,
                      transform.transform.rotation.w);

    tf2::Matrix3x3 m(rotation);
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
