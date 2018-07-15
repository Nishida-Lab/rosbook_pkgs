#include <ros/ros.h>

#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
  // ros init
  ros::init(argc, argv, "rqtplot_example");

  // node handler
  ros::NodeHandle n;

  // publisher
  ros::Publisher pub;
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // publish message
  geometry_msgs::Twist cmd_vel;

  // ros rate
  ros::Rate rate(10);

  // main loop
  while (ros::ok())
  {
    double t = ros::Time::now().toSec();
    cmd_vel.linear.x  = 100.0 * sin(t);
    cmd_vel.linear.y  = 100.0 * sin(1.0 / 2.0 * t);
    cmd_vel.linear.z  = 100.0 * sin(1.0 / 5.0 * t);
    cmd_vel.angular.x = 100.0 * cos(t);
    cmd_vel.angular.y = 100.0 * cos(1.0 / 2.0 * t);
    cmd_vel.angular.z = 100.0 * cos(1.0 / 5.0 * t);
    pub.publish(cmd_vel);
    rate.sleep();
  }

  return 0;
}

