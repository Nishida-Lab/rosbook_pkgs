#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle n;

  tf::TransformBroadcaster tb;
  tf::StampedTransform st;

  ros::Rate r(1.0);

  while(ros::ok())
  {
    tf::Vector3 translation;
    translation.setValue(0, 0, 1.0);
    st.setOrigin(translation);

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);
    st.setRotation(rotation);

    tb.sendTransform(tf::StampedTransform(st, ros::Time::now(), "frame1", "frame2"));
    ROS_INFO("Transform Published");

    r.sleep();
  }

  return 0;
}
