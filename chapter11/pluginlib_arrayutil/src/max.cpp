#include <pluginlib/class_list_macros.h>
#include <pluginlib_arrayutil/max.h>
#include <ros/ros.h>

namespace arrayutil_plugins_max
{
  Max::Max(){}

  double Max::operate()
  {
    if (vec_.size() <= 0)
    {
      ROS_ERROR("array is empty when operation is attempted");
      return -1;
    }

    double max = 0;
    for (std::vector<double>::iterator it = vec_.begin() ; it != vec_.end(); ++it)
    {
      if (it == vec_.begin())
        max = *it;
      
      if (*it > max)
        max = *it;
    } 

    return(max);
  }
}

PLUGINLIB_EXPORT_CLASS(arrayutil_plugins_max::Max, arrayutil_base::ArrayUtil)
