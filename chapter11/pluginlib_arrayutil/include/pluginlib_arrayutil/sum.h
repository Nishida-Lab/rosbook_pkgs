#ifndef PLUGINLIB_ARRAYUTIL_SUM_H_
#define PLUGINLIB_ARRAYUTIL_SUM_H_

#include <pluginlib_arrayutil/arrayutil_base.h>

namespace arrayutil_plugins_sum 
{
  class Sum : public arrayutil_base::ArrayUtil
  {
    public:
      Sum();
      double operate();
  };
}

#endif
