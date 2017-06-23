#include <pluginlib/class_list_macros.h>
#include <pluginlib_arrayutil/sum.h>
#include <ros/ros.h>

namespace arrayutil_plugins_sum
{
  Sum::Sum(){}

  double Sum::operate()
  {
    if (vec_.size() <= 0)
    {
      ROS_ERROR("array is empty when operation is attempted");
      return -1;
    }

    double sum = 0;
    for (std::vector<double>::iterator it = vec_.begin() ; it != vec_.end(); ++it)
    {
      sum += *it;
    } 

    return(sum);
  }
}

PLUGINLIB_EXPORT_CLASS(arrayutil_plugins_sum::Sum, arrayutil_base::ArrayUtil);
