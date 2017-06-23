#include <pluginlib/class_list_macros.h>
#include <pluginlib_arrayutil/min.h>
#include <ros/ros.h>

namespace arrayutil_plugins_min
{
  Min::Min(){}

  double Min::operate()
  {
    if (vec_.size() <= 0)
    {
      ROS_ERROR("array is empty when operation is attempted");
      return -1;
    }

    double min = 10e9;
    for (std::vector<double>::iterator it = vec_.begin() ; it != vec_.end(); ++it)
    {
      if (it == vec_.begin())
        min = *it;
      
      if (*it < min)
        min = *it;
    } 

    return(min);
  }
}

PLUGINLIB_EXPORT_CLASS(arrayutil_plugins_min::Min, arrayutil_base::ArrayUtil)
