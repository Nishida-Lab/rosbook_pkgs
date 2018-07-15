#include <iostream>
#include <random>

#include <ros/ros.h>

#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>


class RosbagExample
{
public:
  RosbagExample();

private:
  void Callback(const std_msgs::Char::ConstPtr& msg);

  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle n;

  double average;
  double data_num;
};


RosbagExample::RosbagExample() :
  average(0),
  data_num(0)
{
  pub = n.advertise<std_msgs::Float64>("/average_num", 1);
  sub = n.subscribe<std_msgs::Char>("/random_num", 1, &RosbagExample::Callback, this);
}

void RosbagExample::Callback(const std_msgs::Char::ConstPtr& msg)
{
  // Publish message
  std_msgs::Float64 pub_msg;

  // calculate the average
  average += (double(msg->data) - average) / (data_num + 1.0);
  pub_msg.data = average;
  data_num++;

  // Publish
  pub.publish(pub_msg);
  ROS_INFO("Published");
}


int main(int argc, char** argv)
{
  // ros init
  ros::init(argc, argv, "rosbag_example_subscriber");

  // class init
  RosbagExample rosbag_example;

  // start to subscribe
  ros::spin();

  return 0;
}

