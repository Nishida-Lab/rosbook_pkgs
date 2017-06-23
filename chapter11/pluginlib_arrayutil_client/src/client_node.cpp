#include <pluginlib_arrayutil_client/pluginlib_arrayutil_client.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    int loop_count = 0;

    while(ros::ok())
    {
      ROS_INFO_STREAM("Loop count: " << ++loop_count);

      pluginlib_arrayutil_client::PluginlibArrayutilClient client;
      client.run();

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("The plugin failed to load for some reason. Error: " << ex.what());
  }

  return 0;
}
