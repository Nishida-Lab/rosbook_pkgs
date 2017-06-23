#include <pluginlib/class_list_macros.h>
#include <pluginlib_arrayutil/ave.h>
#include <ros/ros.h>

namespace arrayutil_plugins_ave
{
  Ave::Ave(){}

  double Ave::operate()
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
    double ave = sum / vec_.size();

    return(ave);
  }
}

PLUGINLIB_EXPORT_CLASS(arrayutil_plugins_ave::Ave, arrayutil_base::ArrayUtil);
