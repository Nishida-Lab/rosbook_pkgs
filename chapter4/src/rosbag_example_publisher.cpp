#include <iostream>
#include <random>

#include <ros/ros.h>

#include <std_msgs/Char.h>


int main(int argc, char** argv)
{
  // ros init
  ros::init(argc, argv, "rosbag_example_publisher");

  // node handler
  ros::NodeHandle n;

  // publisher
  ros::Publisher pub;
  pub = n.advertise<std_msgs::Char>("/random_num", 1);

  // publish message
  std_msgs::Char r_num;

  // for random numbers
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_int_distribution<> rand100(0, 255);

  // ros rate
  ros::Rate rate(10);

  // main loop
  while (ros::ok())
  {
    r_num.data = rand100(mt);
    pub.publish(r_num);
    rate.sleep();
  }

  return 0;
}

