#include "pluginlib_arrayutil/arrayutil_base.h"
#include <ros/ros.h>

namespace arrayutil_base 
{
    void ArrayUtil::setArray(const std::vector<double> vec)
    {
      if(vec.size() <= 0)
      {
          ROS_ERROR("input array is empty when setting");
          return;
      }

      if(vec_.size() > 0)
        vec_.clear();

      vec_ = vec;
    }

    void ArrayUtil::setArray(const double *array, const int size)
    {
      if(!array)
      {
          ROS_ERROR("input array is empty when setting");
          return;
      }

      if(vec_.size() > 0)
        vec_.clear();

      for(int i = 0; i < size; ++i)
        vec_.push_back(array[i]);
    }
}
